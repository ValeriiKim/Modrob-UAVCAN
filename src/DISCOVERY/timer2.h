#ifndef TIMER2_H
#define TIMER2_H

#include "stm32f4xx.h"
#include "stm32f4xx_ll_tim.h"

/** Этот таймер нужен только для отсчёта микросекунд с начала запуска
 * таймера, что необходимо для работы протокола (ноды модуля).
 * Контроллер STM32F407VGT6
 */
namespace timer2
{
    uint64_t volatile micros = 0;
    uint32_t volatile overflow_count = 0;

    void tim2_setup()
    {
        /** (1) Включаем тактирование таймера 2
         *  (2) Установка приоритета прерываний
         *  (3) Включение прерываний
         *  (4) Выбираем величину прескейлера, можно взять его равным 83, тогда
         *      частота таймера будет равна f_clk_CNT = 84 МГц / (83 + 1) = 1 МГц, т.е.
         *      один период сигнала таймера будет равен 1 мкс, т.е. одному тику таймера
         *  (5) Настраиваем режим Upcounting
         *  (6) Выбираем регистра auto-reload так, чтобы период прерывания был равен 1 мс: 1000/f_clk_CNT = 0.001 с
         *  (7) Включаем буферизацию регистра TIM2_ARR (preload feature)
         *  (8) Внутренний источник тактирования таймера
         *  Важно! Частота шины APB1 равна 42 МГц, однако для таймера 2 она удваивается - 84 МГц
         */
        LL_TIM_InitTypeDef TIM_InitStruct = {0};
        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // (1)

        /* TIM2 interrupt Init */
        NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); // (2)
        NVIC_EnableIRQ(TIM2_IRQn); // (3)

        TIM_InitStruct.Prescaler = 83; // (4)
        TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP; // (5)
        TIM_InitStruct.Autoreload = 1000;                   // (6)
        TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
        LL_TIM_Init(TIM2, &TIM_InitStruct);
        LL_TIM_EnableARRPreload(TIM2); // (7)
        LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL); // (8)
        LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
        LL_TIM_DisableMasterSlaveMode(TIM2); 
    }

    void tim2_start()
    {
        // Включение прерываний по переполнению счётчика (когда достигаем значение ARR)
        LL_TIM_EnableIT_UPDATE(TIM2);
        LL_TIM_EnableCounter(TIM2);
    }

    inline uint64_t get_micros()
    {
        // Используем число переполнений, умноженных на число в регистре ARR + текущий счётчик
        return (overflow_count * TIM2->ARR + TIM2->CNT);
    }

    inline void tim2_upcount()
    {
        if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
        {
            LL_TIM_ClearFlag_UPDATE(TIM2);
            overflow_count++;
        }
    }

    // extern "C"
    // {
    //     /* С каждым прерыванием увеличиваем счётчик переполнений */
    //     void TIM2_IRQHandler()
    //     {
    //         if (TIM2->SR & TIM_SR_UIF)
    //         {
    //             TIM2->SR &= ~TIM_SR_UIF;
    //             overflow_count++;
    //         }
    //     }
    // }
}

#endif