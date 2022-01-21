#ifndef MODROB_UAVCAN_NODE_HPP
#define MODROB_UAVCAN_NODE_HPP

// Библиотеки протокола, o1heap и bxcan
#include "canard.h"
#include "o1heap.h"
#include "bxcan.h"
// Библиотека для работы с очередями
#include "cQueue.h"

#include <string.h>
#include <functional>

struct CAN_message
{
    uint64_t microseconds;
    uint32_t extended_can_id;
    size_t payload_size = CANARD_MTU_CAN_CLASSIC;
    uint8_t payload[CANARD_MTU_CAN_CLASSIC] = {0};

    CAN_message()
    {
        microseconds = 0;
        extended_can_id = 0;
        payload_size = CANARD_MTU_CAN_CLASSIC;
        memset(payload, 0, 8);
    }
};

enum class Node_sync_state
{
    state_update = 0,
    state_adjust = 1
};

O1HeapInstance *o1heap_ins;
void *memAllocate(CanardInstance *const ins, const size_t amount)
{
    (void)ins;
    return o1heapAllocate(o1heap_ins, amount);
}
void memFree(CanardInstance *const ins, void *const pointer)
{
    (void)ins;
    o1heapFree(o1heap_ins, pointer);
}

class UAVCAN_node
{
public:
    UAVCAN_node(uint8_t const nodeID, const size_t payload_size = CANARD_MTU_CAN_CLASSIC)
    {
        o1heap_ins = o1heapInit(_base, HEAP_SIZE);
        canard_ins = canardInit(&memAllocate, &memFree);
        tx_queue = canardTxInit(100, payload_size);
        canard_ins.node_id = nodeID; // Set equal to MODULE_ID which is edited in platformio.ini
        q_init(&canrx_queue, sizeof(CAN_message), 32, FIFO, true);
    }

    bool bxCAN_init(const uint32_t peripheral_clock_rate,
                    const uint32_t target_bitrate,
                    std::function<void()> enable_irq_fn,
                    std::function<void()> disable_irq_fn)
    {
        BxCANTimings can_timings;
        bool res = bxCANComputeTimings(peripheral_clock_rate, target_bitrate, &can_timings);
        if (!res)
        {
            // The requested bit rate cannot be set up!
            return res;
        }
        res = bxCANConfigure(0, can_timings, false);
        if (!res)
        {
            // Can't configure bxCAN!
            return res;
        }
        enable_CAN_interrupts = enable_irq_fn;
        disable_CAN_interrupts = disable_irq_fn;
        enable_CAN_interrupts();
        return res;
    }

    bool receive_transfers(std::function<void(const CanardRxTransfer *transfer)> process_transfer)
    {
        CanardRxTransfer transfer;
        while (not q_isEmpty(&canrx_queue))
        {
            CAN_message can_msg;
            CanardFrame rxf;
            disable_CAN_interrupts();
            q_pop(&canrx_queue, &can_msg);
            enable_CAN_interrupts();
            rxf.extended_can_id = can_msg.extended_can_id;
            rxf.payload = can_msg.payload;
            rxf.payload_size = can_msg.payload_size;
            const int8_t res = canardRxAccept(&canard_ins, can_msg.microseconds, &rxf, 0, &transfer, NULL);
            if (res == 1)
            {
                process_transfer(&transfer);
                canard_ins.memory_free(&canard_ins, transfer.payload);
            }
            else if (res < 0)
            {
                // An error has occurred: either an argument is invalid or we've ran out of memory.
                return false;
            }
            else
            {
            }
        }
        return true;
    }

    void send_transmission_queue(uint64_t micros)
    {
        for (const CanardTxQueueItem *item = NULL; (item = canardTxPeek(&tx_queue)) != NULL;) // Look at the top of the TX queue.
        {
            if ((0U == item->tx_deadline_usec) || (item->tx_deadline_usec > micros)) // Check the deadline.
            {
                if (not bxCANPush(0,
                                  micros,
                                  item->tx_deadline_usec,
                                  item->frame.extended_can_id,
                                  item->frame.payload_size,
                                  item->frame.payload))
                {
                    break; // If the driver is busy, break and retry later.
                }
            }
            canard_ins.memory_free(&canard_ins, canardTxPop(&tx_queue, item));
        }
    }

    bool enqueue_transfer(const CanardMicrosecond tx_deadline_us,
                          const CanardPriority priority,
                          const CanardTransferKind transfer_kind,
                          const CanardPortID port_id,
                          const CanardNodeID remote_node_id,
                          const CanardTransferID transfer_id,
                          const size_t payload_size,
                          const void *payload)
    {
        const CanardTransferMetadata transfer_metadata =
            {
                /* .priority       = */ priority,
                /* .transfer_kind  = */ transfer_kind,
                /* .port_id        = */ port_id,
                /* .remote_node_id = */ remote_node_id,
                /* .transfer_id    = */ transfer_id,
            };
        int32_t result = canardTxPush(&tx_queue,
                                      &canard_ins,
                                      tx_deadline_us,
                                      &transfer_metadata,
                                      payload_size,
                                      payload);
        bool const success = (result >= 0);
        return success;
    }

    inline bool canard_IRQhandler(uint64_t micros)
    {
        CAN_message can_msg;
        bool res = bxCANPop(0, &can_msg.extended_can_id, &can_msg.payload_size, can_msg.payload);
        can_msg.microseconds = micros;
        res = q_push(&canrx_queue, &can_msg);
        return res;
    }

    bool rx_subscribe(const CanardTransferKind transfer_kind,
                      const CanardPortID port_id,
                      const size_t extent,
                      const CanardMicrosecond transfer_id_timeout_usec,
                      CanardRxSubscription *const out_subscription)
    {
        int8_t res = canardRxSubscribe(&canard_ins,
                                       transfer_kind,
                                       port_id,
                                       extent,
                                       transfer_id_timeout_usec,
                                       out_subscription);
        if (res < 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /** Функция для подсчёта микросекунд реального времени, должна вызываться в прерывании таймера,
     * которое происходит с периодом 1 мс. 
    **/
    inline void upcount_realtime()
    {
        micros_realtime += 1000;
    }

    inline uint64_t get_realtime()
    {
        return micros_realtime;
    }

    /** Функция принимающая сообщение uavcan.time.Synchronization, должна вызываться как можно ближе
     * к коду, который принимает CAN фрейм с этим сообщением (то есть в прерывании по приёму фреймов)
     **/
    void handle_received_time_sync_message(uint64_t rx_monotonic_timestamp, 
                                           uint64_t previous_transmission_timestamp_us)
    {
        if (state == Node_sync_state::state_adjust)
        {
            uint64_t local_time_phase_error = previous_rx_real_timestamp - previous_transmission_timestamp_us;
            // TODO: adjust_time function
            state = Node_sync_state::state_update;
        }
        else
        {
            previous_rx_real_timestamp = micros_realtime;
            previous_rx_monotonic_timestamp = rx_monotonic_timestamp;
            state = Node_sync_state::state_adjust;
        }
    }

private:
    static constexpr size_t HEAP_SIZE = 4096;
    uint8_t _base[HEAP_SIZE] __attribute__((aligned(O1HEAP_ALIGNMENT))); // Create pointer to the memory arena for the HEAP
    Queue_t canrx_queue;                                                 // queue for recieved CAN frames
    CanardInstance canard_ins;
    CanardTxQueue tx_queue;
    std::function<void()> enable_CAN_interrupts;
    std::function<void()> disable_CAN_interrupts;

    uint64_t micros_realtime = 0; // Milliseconds for realtime synchronization
    Node_sync_state state = Node_sync_state::state_update;
    uint64_t previous_rx_real_timestamp = 0;
    uint64_t previous_rx_monotonic_timestamp = 0;
};

#endif