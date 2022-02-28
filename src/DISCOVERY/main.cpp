#define NUNAVUT_ASSERT(x) assert(x) // объявление макроса для работы serialization.h для DSDL
// Главная тестовая прошивка для отладки различной периферии и протокола UAVCAN

// Библиотеки микроконтроллера
#include "main.hpp"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "timer2.h"

// Библиотеки протокола, o1heap
#include "canard.h"
#include "o1heap.h"
#include "cQueue.h"

// Thin layer above canard and bxcan
#include "modrob_uavcan_node.hpp"

// Заголовочники DSDL (uavcan namespace)
#include "uavcan/node/Heartbeat_1_0.h"

void SystemClock_Config(void);
void CAN_clk_gpio_init();
void bxCAN_interrupts_enable();
void bxCAN_interrupts_disable();

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

		UART_send_float(hb_remote.vendor_specific_status_code, true);
		if (res < 0)
		{
			// usart2::usart_send_string("HERE!\n");
		}
	}
}

UAVCAN_node node(MODULE_ID);

int main()
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	timer2::tim2_setup();
	timer2::tim2_start();
	uint8_t str[] = "USART Transmit DMA\r\n";
	CAN_clk_gpio_init();
	node.bxCAN_init(HAL_RCC_GetPCLK1Freq(), 1000000,
					bxCAN_interrupts_enable,
					bxCAN_interrupts_disable);
	// usart2::UART_send_string("bxCAN controller initialized and started\n");

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
	uint64_t prev_time = timer2::get_micros();

	uavcan_node_Heartbeat_1_0 hb =
		{
			timer2::get_micros() / USEC_IN_SEC,
			uavcan_node_Health_1_0_NOMINAL,
			uavcan_node_Mode_1_0_OPERATIONAL,
			0xA1, // max FF (255)
		};

	static CanardRxSubscription heartbeat_subscription;
	node.rx_subscribe(CanardTransferKindMessage,
					  uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
					  12,
					  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					  &heartbeat_subscription);

	while (1)
	{
		// HAL_Delay(1000);
		// UART_send_string("abcdef");

		// HAL_UART_Transmit_DMA(&huart2, str, sizeof(str) - 1);
		hb.uptime = timer2::get_micros() / USEC_IN_SEC;

		if ((timer2::get_micros() - prev_time) > USEC_IN_SEC)
		{
			// usart2::usart_send_int(node.get_realtime(), true);
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
		// usart2::UART_send_float(0.23, true);
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
	}
	return res;
}

/* Нужно добавлять для обработчиков прерываний обёртку, которая позволяет воспринимать
участок кода как код языка С (НЕ С++). Без этого функции обработчиков
прерываний не хотят работать  */

extern "C"
{
	/**
  * @brief This function handles System tick timer.
  */
	void SysTick_Handler(void)
	{
		HAL_IncTick();
	}
}

// обработчик прерывания от памяти к периферии (USART2-TX) для дебага
extern "C"
{
	void DMA1_Stream6_IRQHandler(void)
	{
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3);
		HAL_DMA_IRQHandler(&hdma_usart2_tx);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
	}
}

extern "C"
{
	/* С каждым прерыванием увеличиваем счётчик переполнений */
	void TIM2_IRQHandler()
	{
		timer2::tim2_upcount();
		// node.upcount_realtime();
	}
}

extern "C"
{
	/* Обработка прерывания по переносу данных из сдвигового регистра в USART_DR при приёме данных по RX */
	void USART2_IRQHandler(void)
	{
		HAL_UART_IRQHandler(&huart2);
		// commander.serial_commander_IRQHandler();
	}
}

/** Обработчик прерывания по приёму CAN фреймов, прерывание происходит, когда
 * в FIFO0 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void CAN1_RX0_IRQHandler(void)
	{
		if (CAN1->RF0R & CAN_RF0R_FMP0) // FMP0 > 0?
		{
			node.canard_IRQhandler(timer2::get_micros());
		}
	}
}

/** Обработчик прерывания по приёму CAN фреймов, прерывание происходит, когда
 * в FIFO1 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
	void CAN1_RX1_IRQHandler(void)
	{
		if (CAN1->RF1R & CAN_RF1R_FMP1) // FMP1 > 0?
		{
			node.canard_IRQhandler(timer2::get_micros());
		}
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{
	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{
	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	}
	LL_SetSystemCoreClock(168000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
	{
		// Error_Handler();
	}
}

void CAN_clk_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* CAN1 interrupt Init */
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
}

void bxCAN_interrupts_enable()
{
	//  Interrupt generated when get new frame in FIFO0 or FIFO1
	SET_BIT(CAN1->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
}

void bxCAN_interrupts_disable()
{
	CLEAR_BIT(CAN1->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
}