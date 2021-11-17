#ifndef MODROB_TEST_MODULE_HPP
#define MODROB_TEST_MODULE_HPP

// Главная тестовая прошивка для отладки различной периферии и протокола

// Библиотеки микроконтроллера
#include "board_config.h"
#include "usart_dma_config.h"
#include "can_config.h"
#include "timer2.h"


// Библиотеки протокола


// Макросы для дебага
#define GREEN_LED_ON    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF   SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON      SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF     SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);

// SerialCommander commander;

int main() {
	board::system_config();
	board::GPIO_Init();
	usart2::usart2_init();
	timer2::tim2_setup();
	timer2::tim2_start();

	usart2::usart_send_string("CAN controller initialized and started\n");

    GREEN_LED_OFF;
	TESTPIN_OFF;





	while(1) 
	{
		// usart2::usart_send_float(var0.get(), 1);
		GREEN_LED_ON;
		LL_mDelay(200);
		GREEN_LED_OFF;
		LL_mDelay(200);

	}

	return 0;
}

/* Нужно добавлять для обработчиков прерываний (пока непонятно для всех
или для только SysTick) обёртку, которая позволяет воспринимать 
участок кода как код языка С (НЕ С++). Без этого функции обработчиков
прерываний не хотят работать  */
extern "C" 
{
    void SysTick_Handler(void) 
    {
    }
}

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





#endif //MODROB_TEST_HPP
