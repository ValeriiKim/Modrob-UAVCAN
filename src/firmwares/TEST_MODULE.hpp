#ifndef MODROB_TEST_MODULE_HPP
#define MODROB_TEST_MODULE_HPP

#define NUNAVUT_ASSERT(x) assert(x) // объявление макроса для работы serialization.h для DSDL
// Главная тестовая прошивка для отладки различной периферии и протокола UAVCAN

// Библиотеки микроконтроллера
#include "board_config.h"
#include "usart_dma_config.h"
#include "timer2.h"

// Библиотеки протокола, o1heap
#include "canard.h"
#include "o1heap.h"
#include "cQueue.h"

// Thin layer above canard and bxcan
#include "modrob_uavcan_node.hpp"

// Заголовочники DSDL (uavcan namespace)
#include "uavcan/node/Heartbeat_1_0.h"

// Макросы для дебага
#define GREEN_LED_ON SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);

constexpr uint32_t USEC_IN_SEC = 1000000; // секунда выраженная в мкс

bool publish_heartbeat(const uavcan_node_Heartbeat_1_0 *const hb, uint64_t micros);

void process_received_transfer(const CanardRxTransfer *transfer)
{
	
	if ((transfer->metadata.transfer_kind == CanardTransferKindMessage) &&
		(transfer->metadata.port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_))
	{
		uavcan_node_Heartbeat_1_0 hb_remote;
		size_t inout_buffer_size_bytes = transfer->payload_size;
		int res = uavcan_node_Heartbeat_1_0_deserialize_(&hb_remote, (uint8_t *)(transfer->payload), &inout_buffer_size_bytes);
		GREEN_LED_ON;
		GREEN_LED_OFF;
		usart2::usart_send_int(hb_remote.vendor_specific_status_code, true);
		if (res < 0)
		{
			// usart2::usart_send_string("HERE!\n");
		}
	}
}

UAVCAN_node node(MODULE_ID);

int main()
{
	board::system_config();
	board::GPIO_Init();
	usart2::usart2_init();
	timer2::tim2_setup();
	timer2::tim2_start();

	board::bxCAN_clock_enable();
	node.bxCAN_init(HAL_RCC_GetPCLK1Freq(), 1000000,
					board::bxCAN_interrupts_enable,
					board::bxCAN_interrupts_disable);
	usart2::usart_send_string("bxCAN controller initialized and started\n");

	GREEN_LED_OFF;
	TESTPIN_OFF;

	uint64_t prev_time = timer2::get_micros();

	uavcan_node_Heartbeat_1_0 hb =
		{
			timer2::get_micros() / USEC_IN_SEC,
			uavcan_node_Health_1_0_NOMINAL,
			uavcan_node_Mode_1_0_OPERATIONAL,
			23, // max FF (255)
		};

	static CanardRxSubscription heartbeat_subscription;
	node.rx_subscribe(CanardTransferKindMessage,
					  uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
					  12,
					  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					  &heartbeat_subscription);

	while (1)
	{
		hb.uptime = timer2::get_micros() / USEC_IN_SEC;

		if ((timer2::get_micros() - prev_time) > USEC_IN_SEC)
		{
			bool res = publish_heartbeat(&hb, timer2::get_micros());
			prev_time = timer2::get_micros();
		}
		node.send_transmission_queue(timer2::get_micros());
		node.receive_transfers(&process_received_transfer);
	}
	return 0;
}

bool publish_heartbeat(const uavcan_node_Heartbeat_1_0 *const hb, uint64_t micros)
{
	static CanardTransferID transfer_id = 0;
	uint8_t payload[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
	size_t inout_buffer_size_bytes = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
	if (uavcan_node_Heartbeat_1_0_serialize_(hb, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS)
	{
		return false; // произошла ошибка сериализации
	}

	bool res = node.enqueue_transfer(micros + 2 * USEC_IN_SEC,
									 CanardPriorityNominal,
									 CanardTransferKindMessage,
									 uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
									 CANARD_NODE_ID_UNSET,
									 transfer_id,
									 inout_buffer_size_bytes,
									 payload);
	++transfer_id;

	if (!res)
	{
		usart2::usart_send_string("Transfer error!\n");
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
	}
	return res;
}

/* Нужно добавлять для обработчиков прерываний обёртку, которая позволяет воспринимать
участок кода как код языка С (НЕ С++). Без этого функции обработчиков
прерываний не хотят работать  */

// обработчик прерывания от памяти к периферии (USART2-TX) для дебага
extern "C"
{
	void DMA1_Channel7_IRQHandler(void)
	{
		usart2::transfer_handler_irq();
	}
}

extern "C"
{
	/* Обработка прерывания по переносу данных из сдвигового регистра в USART_DR при приёме данных по RX */
	void USART2_IRQHandler(void)
	{
		// usart2::receive_handler_irq();
		// commander.serial_commander_IRQHandler();
	}
}

/** Обработчик прерывания по приёму CAN фреймов, прерывание происходит, когда
 * в FIFO0 или FIFO1 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void USB_LP_CAN1_RX0_IRQHandler(void)
	{
		if ((CAN1->RF0R & CAN_RF0R_FMP0) || (CAN1->RF1R & CAN_RF1R_FMP1)) // FMP0 > 0 or FMP1 > 0?
		{
			node.canard_IRQhandler(timer2::get_micros());
		}
	}
}

#endif // MODROB_TEST_HPP
