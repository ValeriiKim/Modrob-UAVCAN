#ifndef MODROB_TEST_MODULE_HPP
#define MODROB_TEST_MODULE_HPP

#define NUNAVUT_ASSERT(x) assert(x) // объявление макроса для работы serialization.h для DSDL
// Главная тестовая прошивка для отладки различной периферии и протокола UAVCAN

// Библиотеки микроконтроллера
#include "board_config.h"
#include "usart_dma_config.h"
#include "can_config.h"
#include "timer2.h"

// Библиотеки протокола, o1heap и bxcan
#include "canard.h"
#include "o1heap.h"
#include "bxcan.h"

// Заголовочники DSDL (uavcan namespace)
#include "uavcan/node/Heartbeat_1_0.h"

// Макросы для дебага
#define GREEN_LED_ON SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);

#define USEC_IN_SEC 1000000 // секунда выраженная в мкс

// размер кучи для работы O1Heap в килобайтах
#define HEAP_SIZE 4096

O1HeapInstance *o1heap_ins;
uint8_t _base[HEAP_SIZE] __attribute__((aligned(O1HEAP_ALIGNMENT))); // Create pointer to the memory arena for the HEAP

static void *memAllocate(CanardInstance *const ins, const size_t amount)
{
	(void)ins;
	return o1heapAllocate(o1heap_ins, amount);
}

static void memFree(CanardInstance *const ins, void *const pointer)
{
	(void)ins;
	o1heapFree(o1heap_ins, pointer);
}

int32_t publish_heartbeat(CanardInstance *const canard, const uavcan_node_Heartbeat_1_0 *const hb, uint64_t micros)
{
	static CanardTransferID transfer_id = 0;
	uint8_t payload[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
	size_t buffer_size_bytes = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
	if (uavcan_node_Heartbeat_1_0_serialize_(hb, payload, &buffer_size_bytes) < NUNAVUT_SUCCESS)
	{
		return -1; // произошла ошибка сериализации
	}
	const CanardTransfer transfer =
		{
			micros + 2 * USEC_IN_SEC,
			CanardPriorityNominal,
			CanardTransferKindMessage,
			uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
			CANARD_NODE_ID_UNSET,
			transfer_id,
			buffer_size_bytes, // sizeof(payload),
			&payload[0],
		};
	++transfer_id;
	int32_t result = canardTxPush(canard, &transfer);
	if (result < 0)
	{
		usart2::usart_send_string("Tranfer error!\n");
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		return result;
	}
}

void send_transmission_queue(CanardInstance *const canard, uint64_t micros)
{
	for (const CanardFrame *txf = NULL; (txf = canardTxPeek(canard)) != NULL;) // Look at the top of the TX queue.
	{
		if ((0U == txf->timestamp_usec) || (txf->timestamp_usec > micros)) // Check the deadline.
		{
			if (bxCANPush(0,
						  micros,
						  txf->timestamp_usec,
						  txf->extended_can_id,
						  txf->payload_size,
						  txf->payload))
			{
				canardTxPop(canard);
				canard->memory_free(canard, (CanardFrame *)txf); // Remove the frame from the queue after it's transmitted.
			}
		}
	}
}

int main()
{
	board::system_config();
	board::GPIO_Init();
	usart2::usart2_init();
	timer2::tim2_setup();
	timer2::tim2_start();

	board::bxCAN_clock_enable();
	BxCANTimings can_timings;
	// bxCAN находится на APB1 - частота PCLK1 должна быть равна 36 МГц
	bool res = bxCANComputeTimings(HAL_RCC_GetPCLK1Freq(), 1000000, &can_timings);
	if (!res)
	{
		usart2::usart_send_string("The requested bit rate cannot be set up!\n");
	}
	res = bxCANConfigure(0, can_timings, false);
	if (!res)
	{
		usart2::usart_send_string("Can't configure bxCAN!\n");
	}
	usart2::usart_send_string("bxCAN controller initialized and started\n");

	GREEN_LED_OFF;
	TESTPIN_OFF;

	o1heap_ins = o1heapInit(_base, HEAP_SIZE);

	CanardInstance canard_ins = canardInit(&memAllocate, &memFree);
	canard_ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC; // Here we use classic CAN
	canard_ins.node_id = MODULE_ID;				   // Set equal to MODULE_ID which is edited in platformio.ini

	uint64_t prev_time = timer2::get_micros();
	// uavcan_node_Heartbeat_1_0 hb;
	// hb.uptime = timer2::get_micros() / USEC_IN_SEC;
	// hb.health.value = uavcan_node_Health_1_0_NOMINAL;
	// hb.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
	// hb.vendor_specific_status_code = 0x23;

	uavcan_node_Heartbeat_1_0 hb =
	{
		timer2::get_micros() / USEC_IN_SEC,
		uavcan_node_Health_1_0_NOMINAL,
		uavcan_node_Mode_1_0_OPERATIONAL,
		0x23U,
	};

	while (1)
	{
		// usart2::usart_send_float(var0.get(), 1);
		hb.uptime = timer2::get_micros() / USEC_IN_SEC;
		hb.health.value = uavcan_node_Health_1_0_CAUTION;
		hb.mode.value = uavcan_node_Mode_1_0_SOFTWARE_UPDATE;
		hb.vendor_specific_status_code = 0x23;
		
		if ((timer2::get_micros() - prev_time) > USEC_IN_SEC)
		{
			publish_heartbeat(&canard_ins, &hb, timer2::get_micros());
			prev_time = timer2::get_micros();
		}
		send_transmission_queue(&canard_ins, timer2::get_micros());
	}

	return 0;
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
 * в FIFO0 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void USB_LP_CAN1_RX0_IRQHandler(void)
	{
		if (CAN1->RF0R & CAN_RF0R_FMP0) // FMP0 > 0?
		{
		}
	}
}

#endif // MODROB_TEST_HPP
