#define NUNAVUT_ASSERT(x) assert(x) // объявление макроса для работы serialization.h для DSDL
#include <stdlib.h>
// STM32F303RE drivers
#include "main.hpp"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "timer2.h"

// libcanard, o1heap and cQueue libs
#include "canard.h"
#include "o1heap.h"
#include "cQueue.h"

// Thin layer above canard and bxcan
#include "modrob_uavcan_node.hpp"

#include "algorithm"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void CAN_clk_gpio_init();
void bxCAN_interrupts_enable();
void bxCAN_interrupts_disable();

// DSDL Headers (uavcan namespace)
#include "uavcan/node/Heartbeat_1_0.h"
// #include "modrob/sensor_module/sensor_data_0_1.h"
#include "modrob/sensor_module/sensor_data_0_2.h"

constexpr uint32_t USEC_IN_SEC = 1000000; // секунда выраженная в мкс

bool publish_heartbeat(const uavcan_node_Heartbeat_1_0 *const hb, uint64_t micros);
bool publish_sensor_data(const modrob_sensor_module_sensor_data_0_2 *const sdata, uint64_t micros);
void process_received_transfer(const CanardRxTransfer *transfer)
{

    if ((transfer->metadata.transfer_kind == CanardTransferKindMessage) &&
        (transfer->metadata.port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_))
    {
        uavcan_node_Heartbeat_1_0 hb_remote;
        size_t inout_buffer_size_bytes = transfer->payload_size;
        int res = uavcan_node_Heartbeat_1_0_deserialize_(&hb_remote, (uint8_t *)(transfer->payload), &inout_buffer_size_bytes);
        usart2::UART_send_float(hb_remote.vendor_specific_status_code, true);
        if (res < 0)
        {
        }
    }
}

extern TIM_HandleTypeDef htim3;
UAVCAN_node node(MODULE_ID);

int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    CAN_clk_gpio_init();
    MX_DMA_Init();
    usart2::USART2_UART_Init();
    usart2::DMA_UART_LinkageInit();
    timer2::tim2_setup();
    timer2::tim2_start();
    srand(HAL_GetTick());

    node.bxCAN_init(HAL_RCC_GetPCLK1Freq(), 1000000,
                    bxCAN_interrupts_enable,
                    bxCAN_interrupts_disable);
    usart2::UART_send_string("All peripherals successfully inited");
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);

    uint64_t prev_time = timer2::get_micros();
    uint64_t next_1_hz_timestep = USEC_IN_SEC;
    uint64_t next_20_hz_timestep = 50000;

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

    modrob_sensor_module_sensor_data_0_2 sensor_data = {};
    sensor_data.cur_pos[0] = 0;
    sensor_data.cur_pos[1] = 0;
    // Формируем статические данные о подвижных препятствиях
    sensor_data.obstacles.count = 10;
    sensor_data.obstacles.elements[0].obst_pos[0] = 1;
    sensor_data.obstacles.elements[0].obst_pos[1] = 1;
    sensor_data.obstacles.elements[0].obst_vel[0] = -0.3;
    sensor_data.obstacles.elements[0].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[0].radius = 0.4;

    sensor_data.obstacles.elements[1].obst_pos[0] = 1.5;
    sensor_data.obstacles.elements[1].obst_pos[1] = 1;
    sensor_data.obstacles.elements[1].obst_vel[0] = -0.6;
    sensor_data.obstacles.elements[1].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[1].radius = 0.3;

    sensor_data.obstacles.elements[2].obst_pos[0] = 1;
    sensor_data.obstacles.elements[2].obst_pos[1] = -1;
    sensor_data.obstacles.elements[2].obst_vel[0] = -0.2;
    sensor_data.obstacles.elements[2].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[2].radius = 0.4;

    sensor_data.obstacles.elements[3].obst_pos[0] = 1.5;
    sensor_data.obstacles.elements[3].obst_pos[1] = 1.5;
    sensor_data.obstacles.elements[3].obst_vel[0] = -0.5;
    sensor_data.obstacles.elements[3].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[3].radius = 0.2;

    sensor_data.obstacles.elements[4].obst_pos[0] = 0.2;
    sensor_data.obstacles.elements[4].obst_pos[1] = 1;
    sensor_data.obstacles.elements[4].obst_vel[0] = -0.3;
    sensor_data.obstacles.elements[4].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[4].radius = 0.4;

    sensor_data.obstacles.elements[5].obst_pos[0] = -1;
    sensor_data.obstacles.elements[5].obst_pos[1] = -1;
    sensor_data.obstacles.elements[5].obst_vel[0] = 0.3;
    sensor_data.obstacles.elements[5].obst_vel[1] = 0.2;
    sensor_data.obstacles.elements[5].radius = 0.4;

    sensor_data.obstacles.elements[6].obst_pos[0] = -1;
    sensor_data.obstacles.elements[6].obst_pos[1] = -1.5;
    sensor_data.obstacles.elements[6].obst_vel[0] = 0.4;
    sensor_data.obstacles.elements[6].obst_vel[1] = 0.2;
    sensor_data.obstacles.elements[6].radius = 0.2;

    sensor_data.obstacles.elements[7].obst_pos[0] = 1.8;
    sensor_data.obstacles.elements[7].obst_pos[1] = 0;
    sensor_data.obstacles.elements[7].obst_vel[0] = -0.3;
    sensor_data.obstacles.elements[7].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[7].radius = 0.4;

    sensor_data.obstacles.elements[8].obst_pos[0] = -2;
    sensor_data.obstacles.elements[8].obst_pos[1] = 0;
    sensor_data.obstacles.elements[8].obst_vel[0] = 0.3;
    sensor_data.obstacles.elements[8].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[8].radius = 0.4;

    sensor_data.obstacles.elements[9].obst_pos[0] = 0;
    sensor_data.obstacles.elements[9].obst_pos[1] = -2;
    sensor_data.obstacles.elements[9].obst_vel[0] = -0.3;
    sensor_data.obstacles.elements[9].obst_vel[1] = -0.2;
    sensor_data.obstacles.elements[9].radius = 0.4;

    sensor_data.segments.count = 20;

    // float rand_val;
    while (1)
    {

        hb.uptime = timer2::get_micros() / USEC_IN_SEC;
        // usart2::UART_send_float(val, true);
        if (timer2::get_micros() >= next_1_hz_timestep)
        {
            next_1_hz_timestep += USEC_IN_SEC;
            // usart2::usart_send_int(node.get_realtime(), true);
            bool res = publish_heartbeat(&hb, timer2::get_micros());
        }
    

        if (timer2::get_micros() >= next_20_hz_timestep)
        {
            // генерируем набор случайных координат в диапазоне 0.5...3 метра
            /* генерируем набор случайных отрезков:
            x1 = [1...3], y1 = [-2...2], x2 = x1 + [-0.5...0.5], y2 = y1 + [-0.5...0.5] 
            */
            for (size_t i = 0; i < sensor_data.segments.count; i++)
            {
                sensor_data.segments.elements[i].x1 = (rand() % 200 + 100) / 100.0;
                sensor_data.segments.elements[i].y1 = (rand() % 400 - 200) / 100.0;
                sensor_data.segments.elements[i].x2 = sensor_data.segments.elements[i].x1 + (rand() % 100 - 50) / 100.0;
                sensor_data.segments.elements[i].y2 = sensor_data.segments.elements[i].y1 + (rand() % 100 - 50) / 100.0;
            }
            next_20_hz_timestep += 50000;
            publish_sensor_data(&sensor_data, timer2::get_micros());
        }
        node.send_transmission_queue(timer2::get_micros());
        node.receive_transfers(&process_received_transfer);
    }
    /* USER CODE END 3 */
}

// Публикация сенсорных данных на шину
bool publish_sensor_data(const modrob_sensor_module_sensor_data_0_2 *const sdata, uint64_t micros)
{
    static CanardTransferID transfer_id = 0;
    uint8_t payload[modrob_sensor_module_sensor_data_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
    size_t inout_buffer_size_bytes = modrob_sensor_module_sensor_data_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_;
    if (modrob_sensor_module_sensor_data_0_2_serialize_(sdata, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS)
    {
        return false;
        usart2::UART_send_string("Serialization error!\n");
    }
    bool res = node.enqueue_transfer(micros + 50200,
                                     CanardPriorityNominal,
                                     CanardTransferKindMessage,
                                     150,
                                     CANARD_NODE_ID_UNSET,
                                     transfer_id,
                                     inout_buffer_size_bytes,
                                     payload);
    ++transfer_id;
    if (!res)
    {
        usart2::UART_send_string("Transfer sensor data error!\n");
        // An error has occurred: either an argument is invalid or we've ran out of memory.
        // It is possible to statically prove that an out-of-memory will never occur for a given application if the
        // heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
    }
    return res;
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
        usart2::UART_send_string("Transfer error!\n");
        // An error has occurred: either an argument is invalid or we've ran out of memory.
        // It is possible to statically prove that an out-of-memory will never occur for a given application if the
        // heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
    }
    return res;
}

/** Обработчик прерывания по приёму CAN фреймов, прерывание происходит, когда
 * в FIFO0 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
    void USB_LP_CAN_RX0_IRQHandler(void)
    {
        // LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        if (CAN->RF0R & CAN_RF0R_FMP0) // FMP0 > 0
        {
            node.canard_IRQhandler(timer2::get_micros());
        }
        // LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
}

/** Обработчик прерывания по приёму CAN фреймов, прерывание происходит, когда
 * в FIFO1 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */
extern "C"
{
    void CAN_RX1_IRQHandler(void)
    {
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        if (CAN->RF1R & CAN_RF1R_FMP1) // FMP1 > 0?
        {
            node.canard_IRQhandler(timer2::get_micros());
        }
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
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

    // __HAL_RCC_GPIOB_CLK_ENABLE();
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
    // Установка приоритета и включение прерывания по получению сообщения в FIFO0
    NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
    // Установка приоритета и включение прерывания по получению сообщения в FIFO1
    NVIC_SetPriority(CAN_RX1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    NVIC_EnableIRQ(CAN_RX1_IRQn);
}

void bxCAN_interrupts_enable()
{
    //  Interrupt generated when get new frame in FIFO0 or FIFO1
    SET_BIT(CAN->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
}

void bxCAN_interrupts_disable()
{
    CLEAR_BIT(CAN->IER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
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
