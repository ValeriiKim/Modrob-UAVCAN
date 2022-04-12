#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// Библиотеки микроконтроллера stm32f103c8t6
#include "stm32f1xx.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

#define SYSCLOCK 72000000U	 // максимальная частота контроллера Bluepill
#define CAN_BITRATE 1000000U // скорость передачи данных по шине CAN

namespace board
{
	/**
	 * @brief System Clock Configuration (настройка частоты тактирования микроконтроллера на 72 МГц).
	 * SysTick setup for ticks with 1 ms period
	 * @retval None
	 */
	void system_clock_config(void)
	{
		LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
		while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
		{
		}
		LL_RCC_HSE_Enable();

		/* Wait till HSE is ready */
		while (LL_RCC_HSE_IsReady() != 1)
		{
		}
		LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
		LL_RCC_PLL_Enable();

		/* Wait till PLL is ready */
		while (LL_RCC_PLL_IsReady() != 1)
		{
		}
		LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
		LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
		LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

		/* Wait till System clock is ready */
		while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
		{
		}
		LL_Init1msTick(SYSCLOCK);
		LL_SetSystemCoreClock(SYSCLOCK);
	}

	/** Общая конфигурация системы: включение тактирования AFIO, настройка портов для прошивания,
	 * вызов функции настройки частоты тактирования на 72 МГц.
	 * Включение прерываний для CAN
	 */
	void system_config(void)
	{
		// включение тактирования AFIO
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

		/* System interrupt init*/
		NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

		/** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
		 */
		LL_GPIO_AF_Remap_SWJ_NOJTAG();

		system_clock_config();

		// Прерывание по получению сообщения в FIFO0
		// Установка приоритета и включение прерываний
		NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
		NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
		// Прерывание по получению сообщения в FIFO1
		// Установка приоритета и включение прерываний
		NVIC_SetPriority(CAN1_RX1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
		NVIC_EnableIRQ(CAN1_RX1_IRQn);
	}

	void GPIO_Init(void)
	{
		LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

		/* GPIO Ports Clock Enable */
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

		// Pins for DEBUG
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

		/**/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/**/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// PB8   ------> CAN_RX
		// PB9   ------> CAN_TX
		/* CAN-RX
		 */
		GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* CAN-TX
		 */
		GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/**/
		__HAL_AFIO_REMAP_CAN1_2();
	}
	/** Enable the clock of the CAN peripheral
	 */
	void bxCAN_clock_enable()
	{
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN);
		// Предполагается, что для libcanard остальные настройки будут сделаны в bxCAN.h
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

} // namespace board

#endif