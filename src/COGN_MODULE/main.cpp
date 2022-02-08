/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.hpp"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "timer2.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void CAN_clk_gpio_init();

extern TIM_HandleTypeDef htim3;

int main(void)
{

    /* MCU Configuration--------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    usart2::USART2_UART_Init();
    usart2::DMA_UART_LinkageInit();
    timer2::tim2_setup();
    timer2::tim2_start();
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);

    while (1)
    {
        HAL_Delay(500);
        // UART_send_float(timer2::get_micros() / 1000000.0, true);
        if (usart2::temp_buffer[0] == 'a')
        {
            usart2::UART_send_string("dfdfdfdfd");
        }
    }
    /* USER CODE END 3 */
}

extern "C"
{
    /**
  * @brief This function handles TIM3 global interrupt.
  */
    void TIM3_IRQHandler(void)
    {
        // LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        HAL_TIM_IRQHandler(&htim3);
        // LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
}

extern "C"
{
    /**
  * @brief This function handles TIM2 global interrupt.
  */
    void TIM2_IRQHandler(void)
    {
        timer2::tim2_upcount();
        // UART_send_float(timer2::get_micros()/1000000.0, true);
    }
}

extern "C"
{
    /**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
    void DMA1_Channel7_IRQHandler(void)
    {
        usart2::transfer_handler_irq();
    }
}

extern "C"
{
    /**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
    void USART2_IRQHandler(void)
    {
        // LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        usart2::receive_handler_irq();
        // LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
    {
    }
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_9, LL_RCC_PREDIV_DIV_1);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }
    LL_SetSystemCoreClock(72000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
        Error_Handler();
    }
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

void CAN_clk_gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8------> CAN_RX
    PB9------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
