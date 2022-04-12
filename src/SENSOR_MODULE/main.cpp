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
#include "modrob/sensor_module/sensor_data_0_1.h"

constexpr uint32_t USEC_IN_SEC = 1000000; // секунда выраженная в мкс

bool publish_heartbeat(const uavcan_node_Heartbeat_1_0 *const hb, uint64_t micros);
bool publish_sensor_data(const modrob_sensor_module_sensor_data_0_1 *const sdata, uint64_t micros);
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

    modrob_sensor_module_sensor_data_0_1 sensor_data = {};
    sensor_data.cur_pos[0] = 0.2;
    sensor_data.cur_pos[1] = 0.3;
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

    sensor_data.scanX.count = 36;
    sensor_data.scanY.count = 36;
    // Набор точек скана со статическими данными
    float scan_x[] = {2, 1.99606, 1.98425, 1.96461, 1.93723, 1.90221, 1.85969, 1.80984, 1.75286, 1.68896,
                      1.61841, 1.54147, 1.45846, 1.3697, 1.27554, 1.17634, 1.07251, 0.964456, 0.852596, 0.737374,
                      0.619245, 0.498675, 0.376139, 0.252119, 0.127106, 0.00159183, -0.123929, -0.248961, -0.373011, -0.495591,
                      -0.616217, -0.734414, -0.849715, -0.961666, -1.06982, -1.17377, -1.27308, -1.36738, -1.45628, -1.53944,
                      -1.61653, -1.68725, -1.75132, -1.80849, -1.85852, -1.90123, -1.93644, -1.96401, -1.98384, -1.99585, -2, -1.99625,
                      -1.98464, -1.9652, -1.93802, -1.90319, -1.86086, -1.8112, -1.75439, -1.69067, -1.62028, -1.5435, -1.46064, -1.37202,
                      -1.27799, -1.17892, -1.0752, -0.967249, -0.85548, -0.740338, -0.622277, -0.501764, -0.379271, -0.255284, -0.13029,
                      -0.00478265, 0.120744, 0.245794, 0.369875, 0.492498, 0.61318, 0.731444, 0.846824, 0.958865, 1.06713, 1.17118, 1.27062,
                      1.36504, 1.45409, 1.5374, 1.61465, 1.68554, 1.74978, 1.80712, 1.85734, 1.90023, 1.93563, 1.96341, 1.98344, 1.99565};
    float scan_y[] = {0, 0.125517, 0.25054, 0.374575, 0.497133, 0.617731, 0.735894, 0.851155, 0.963061, 1.07117,
                      1.17506, 1.27431, 1.36854, 1.45737, 1.54046, 1.61747, 1.68811, 1.75209, 1.80917, 1.85911,
                      1.90172, 1.93683, 1.96431, 1.98405, 1.99596, 2, 1.99616, 1.98444, 1.96491, 1.93762, 1.9027, 1.86028,
                      1.81052, 1.75362, 1.68981, 1.61934, 1.54249, 1.45955, 1.37086, 1.27676, 1.17763, 1.07386, 0.965851, 0.854037,
                      0.738855, 0.62076, 0.500218, 0.377704, 0.253701, 0.128697, 0.00318653, -0.122337, -0.247378, -0.371444, -0.494045,
                      -0.614699, -0.732929, -0.84827, -0.960266, -1.06848, -1.17247, -1.27185, -1.36621, -1.45518, -1.53842, -1.61559,
                      -1.6864, -1.75055, -1.8078, -1.85793, -1.90073, -1.93604, -1.96371, -1.98364, -1.99575, -1.99999, -1.99635, -1.98484,
                      -1.9655, -1.93841, -1.90368, -1.86145, -1.81187, -1.75516, -1.69152, -1.62121, -1.54452, -1.46173, -1.37318, -1.27922,
                      -1.18021, -1.07655, -0.968645, -0.856922, -0.74182, -0.623794, -0.503308, -0.380838, -0.256867, -0.131883};
    // std::copy(scan_x, scan_x + 36, sensor_data.scanX.elements);
    // std::copy(scan_y, scan_y + 36, sensor_data.scanY.elements);
    float rand_val;
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
            for (size_t i = 0; i < sensor_data.scanX.count; i++)
            {
                rand_val = (rand() % 250 + 50) / 100.0;
                sensor_data.scanX.elements[i] = rand_val;
            }
            for (size_t j = 0; j < sensor_data.scanY.count; j++)
            {
                rand_val = (rand() % 250 + 50) / 100.0;
                sensor_data.scanY.elements[j] = rand_val;
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
bool publish_sensor_data(const modrob_sensor_module_sensor_data_0_1 *const sdata, uint64_t micros)
{
    static CanardTransferID transfer_id = 0;
    uint8_t payload[modrob_sensor_module_sensor_data_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
    size_t inout_buffer_size_bytes = modrob_sensor_module_sensor_data_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    if (modrob_sensor_module_sensor_data_0_1_serialize_(sdata, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS)
    {
        return false;
        usart2::UART_send_string("Serialization error!\n");
    }
    bool res = node.enqueue_transfer(micros + 50000,
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
