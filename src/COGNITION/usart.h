#ifndef __USART_H__
#define __USART_H__

#include "main.hpp"
#include "printf.h"
#include <string.h>
#include "lwrb.h"

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

void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (uartHandle->Instance == USART2)
    {
        /* USER CODE BEGIN USART2_MspInit 0 */

        /* USER CODE END USART2_MspInit 0 */
        /* USART2 clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 DMA Init */
        /* USART2_TX Init */
        hdma_usart2_tx.Instance = DMA1_Stream6;
        hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
        {
            
        }

        __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart2_tx);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        /* USER CODE BEGIN USART2_MspInit 1 */

        /* USER CODE END USART2_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

    if (uartHandle->Instance == USART2)
    {
        /* USER CODE BEGIN USART2_MspDeInit 0 */

        /* USER CODE END USART2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

        /* USART2 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmatx);

        /* USART2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
        /* USER CODE BEGIN USART2_MspDeInit 1 */

        /* USER CODE END USART2_MspDeInit 1 */
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
HAL_StatusTypeDef UART_send_float(float value, bool newline)
{
    int result;
    char temp_buffer[128];
    char new_line[] = "\n";
    size_t newline_size = 0;

    result = snprintf_(temp_buffer, sizeof(temp_buffer), "%.4f", value);
    if (result < 0)
    {
        return HAL_StatusTypeDef::HAL_ERROR; // копирование в буфер прошло неудачно, это ошибка
    }
    if (newline) // если хотим каждый символ печатать с новой строки
    {
        strcat(temp_buffer, new_line); // добавляем символ \n
        newline_size = 1;              // указываем сколько нужно ещё байт отправить
    }
    return HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_buffer, result + newline_size);
}

/** Отправка константной строки по USART2 с помощью DMA, однако несмотря на это DMA, команда 
 * блокирующая, т.е. она ждёт, пока передаваемая строка полностью не отправится
*@param str указатель на строку  
 */
void UART_send_string(const char *str)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, strlen(str));
}

#endif