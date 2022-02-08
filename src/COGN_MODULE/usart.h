/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

/* Includes ------------------------------------------------------------------*/
#include "main.hpp"
#include "printf.h"
#include <string.h>
#include "lwrb.h"

namespace usart2
{
    UART_HandleTypeDef huart2;
    DMA_HandleTypeDef hdma_usart2_tx;

    constexpr uint16_t MAX_TX_LEN = 256;
    constexpr uint16_t MAX_RX_LEN = 256;
    // Переменные для работы с буфером UART (DMA TX и приём RX)
    // Набор переменных для буфера передачи TX
    static lwrb_t usart_dma_tx_buff;
    static uint8_t usart_dma_tx_buff_data[MAX_TX_LEN];
    static size_t usart_dma_tx_len;
    // Набор переменных для буфера приёма RX
    static lwrb_t usart_rx_ringbuf;
    static uint8_t usart_rx_ringbuf_data[MAX_RX_LEN];
    // Вспомогательный буфер для получения байтов после фиксации лимитирующего символа
    uint8_t temp_buffer[MAX_RX_LEN];

    uint8_t lim_symbol = '\n';
    // Флаг, показывающий, что достигнут лимитирующий символ и строка скопирована в буфер
    bool rx_data_copied = false;

    /* USART2 init function */
    void USART2_UART_Init(void)
    {
        LL_USART_InitTypeDef USART_InitStruct = {0};
        LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        /**USART2 GPIO Configuration
     *PA2------> USART2_TX
     *PA3------> USART2_RX
  */
        GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 DMA Init */

        /* USART2_TX Init */
        LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);
        LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

        /* USART2 interrupt Init */
        NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        NVIC_EnableIRQ(USART2_IRQn);
        /* Включаем прерывания RXNE, которые генерируются если ORE=1 или RXNE=1 
	в регистре статуса USART_SR, т.е соответственно при переполнении (ORE) и когда
	принятые данные готовы к чтению (RXNE) */
        LL_USART_EnableIT_RXNE(USART2);

        USART_InitStruct.BaudRate = 115200;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
        USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
        USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
        LL_USART_Init(USART2, &USART_InitStruct);
        LL_USART_DisableIT_CTS(USART2);
        LL_USART_ConfigAsyncMode(USART2);
        LL_USART_Enable(USART2);
    }

    void DMA_UART_LinkageInit()
    {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
        LL_USART_EnableDMAReq_TX(USART2);
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_7,
                                LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_TRANSMIT));
        // Enable Channel 7 Transfer complete interrupt and Transfer error interrupt
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
        LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_7);
        // Clear Channel 7 global interrupt flag, transfer complete flag and transfer error flag
        LL_DMA_ClearFlag_GI7(DMA1);
        LL_DMA_ClearFlag_TC7(DMA1);
        LL_DMA_ClearFlag_TE7(DMA1);

        lwrb_init(&usart_dma_tx_buff, usart_dma_tx_buff_data, sizeof(usart_dma_tx_buff_data));
        lwrb_init(&usart_rx_ringbuf, usart_rx_ringbuf_data, sizeof(usart_rx_ringbuf_data));
    }

    inline bool UART_start_DMA_TxTransfer()
    {
        uint32_t old_primask;
        bool start_flag = false;

        old_primask = __get_PRIMASK();
        __disable_irq();
        // проверяем, что передача не активна в настоящий момент

        if (usart_dma_tx_len == 0)
        {
            // проверяем, есть ли что отправлять
            usart_dma_tx_len = lwrb_get_linear_block_read_length(&usart_dma_tx_buff);
            if (usart_dma_tx_len > 0)
            {
                // создаём указатель на первый элемент в линейном блоке буфера на передачу
                void *ptr = lwrb_get_linear_block_read_address(&usart_dma_tx_buff);
                LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
                LL_DMA_ClearFlag_GI7(DMA1);
                LL_DMA_ClearFlag_TC7(DMA1);
                LL_DMA_ClearFlag_TE7(DMA1);
                // Начинаем DMA передачу
                LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, usart_dma_tx_len);
                // устанавливаем адрес памяти для передачи
                LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)ptr);
                LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
                start_flag = true;
            }
        }
        __set_PRIMASK(old_primask);
        return start_flag;
    }

    /** Функция обрабатывающая прерывание по завершению передачи по DMA от памяти
 * к регистру данных USART2. Эта функция реализована конкретно для 7 канала DMA и
 * она должна быть вызвана в void DMA1_Channel7_IRQHandler(void)
 **/
    inline void transfer_handler_irq()
    {
        if (LL_DMA_IsActiveFlag_TC7(DMA1))
        {
            LL_DMA_ClearFlag_TC7(DMA1);
            lwrb_skip(&usart_dma_tx_buff, usart_dma_tx_len);
            usart_dma_tx_len = 0;
            // пробуем отправить ещё данные, если они есть
            UART_start_DMA_TxTransfer();
        }
        else if (LL_DMA_IsActiveFlag_TE7(DMA1))
        {
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
        }
    }

    /* Обработка прерывания по получению данных по RX. Функция должна быть вызвана
внутри функции void USART2_IRQHandler(void) 
 */
    inline void receive_handler_irq()
    {
        // произошло прерывание по получению данных: RXNE = 1
        if (LL_USART_IsActiveFlag_RXNE(USART2))
        {
            uint8_t temp_byte[1];
            temp_byte[0] = LL_USART_ReceiveData8(USART2); // получаем байт из USART
            lwrb_write(&usart_rx_ringbuf, temp_byte, 1);  // записываем байт в буфер приёма
            // проверяем, получили ли мы лимитирующий символ строки, обычно это \n
            if ((temp_byte[0] == lim_symbol))
            {
                // переносим байты из главного буфера usart во временный буфер
                lwrb_read(&usart_rx_ringbuf, temp_buffer, sizeof(temp_buffer));
                // устанавливаем флаг, что данные перенесены (сброс флага, после очистки буфера)
                rx_data_copied = true;
            }
        }
        // произошла ошибка по переполнению ORE = 1
        if (LL_USART_IsActiveFlag_ORE(USART2))
        {
            // Clear by writing 1 to the ORECF, in the USART_ICR register.
            LL_USART_ClearFlag_ORE(USART2);
        }
    }

    /** Отправка числа с плавающей точкой (float) по USART2 с помощью DMA, используется функция библиотеки HAL
*@param value значение float, которое нужно отправить
*@param newline логическое значение, позволяющее отправить число с новой строки, если
newline равно true
*@return HAL_StatusTypeDef - результат выполнения функции HAL_UART_Transmit_DMA
 */
    bool UART_send_float(float value, bool newline)
    {
        int result;
        char temp_buffer[32];
        char new_line[] = "\n";
        size_t newline_size = 0;

        result = snprintf_(temp_buffer, sizeof(temp_buffer), "%.4f", value);
        if (result < 0)
        {
            return false; // копирование в буфер прошло неудачно, это ошибка
        }
        if (newline) // если хотим каждый символ печатать с новой строки
        {
            strcat(temp_buffer, new_line); // добавляем символ \n
            newline_size = 1;              // указываем сколько нужно ещё байт отправить
        }
        while (not LL_USART_IsActiveFlag_TC(USART2))
        {
        }
        LL_USART_ClearFlag_TC(USART2);
        lwrb_write(&usart_dma_tx_buff, temp_buffer, result + newline_size);
        UART_start_DMA_TxTransfer();
        return true;
    }

    /** Отправка константной строки по USART2 с помощью DMA, однако несмотря на это DMA, команда 
 * блокирующая, т.е. она ждёт, пока передаваемая строка полностью не отправится
*@param str указатель на строку  
 */
    void UART_send_string(const char *str)
    {
        // Записываем новые данные в буфер, только если предыдущая передача по USART уже закончилась
        while (not LL_USART_IsActiveFlag_TC(USART2))
        {
        }
        LL_USART_ClearFlag_TC(USART2);
        size_t len = strlen(str);
        if (lwrb_get_free(&usart_dma_tx_buff) >= len)
        {
            lwrb_write(&usart_dma_tx_buff, str, strlen(str));
            UART_start_DMA_TxTransfer();
        }

        // }
    }

    /* Функция для передачи полученных по RX байт в TX (режим loop-back)
 */
    // void usart_echo()
    // {
    //     uint8_t temp_buffer[128];
    //     // Ждём пока предыдущая передача по TX не закончится (пока не установится флаг)
    //     while (not LL_USART_IsActiveFlag_TC(USART2))
    //     {
    //     }
    //     LL_USART_ClearFlag_TC(USART2);
    //     // if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
    //     // {
    //     // USART2->SR &= ~USART_SR_TC;
    //     // получаем число байт линейной части буфера, которые можно читать
    //     // size_t len = lwrb_get_linear_block_read_length(&usart_rx_ringbuf);
    //     size_t len = lwrb_get_full(&usart_rx_ringbuf);
    //     // читаем байты из буфера RX и переписываем их в temp_buffer
    //     lwrb_read(&usart_rx_ringbuf, temp_buffer, len);
    //     // пишем эти байты в буфер для отправки через DMA
    //     lwrb_write(&usart_dma_tx_buff, temp_buffer, len);
    //     usart_start_tx_dma_transfer();
    //     // }
    // }
}

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
