/*
 * usart.c
 *
 *  Created on: Jul 26, 2025
 *      Author: prate
 */
#include <stdint.h>
#include "usart.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

#define CR1_UE        (1U << 13)
#define CR1_TE        (1U << 3)
#define CR1_RE        (1U << 2)

#define SR_RXNE	      (1U << 5)
#define SR_TXE        (1U << 7)

#define RCC_USART1_EN (1U << 4)

#define SYSTEM_FREQUENCY 16000000
#define PERIPHERAL_CLOCK SYSTEM_FREQUENCY
#define BAUDRATE         115200

void usart_init(void){

	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	usart_set_baudrate(USART2, PERIPHERAL_CLOCK, BAUDRATE);

	USART2->CR1 |= USART_CR1_TE;
	USART2->CR1 |= USART_CR1_RE;

	//Set M = 0 for 8 bit packets
	USART2->CR1 &= ~USART_CR1_M;
	//1 Stop bit
	USART2->CR2 &= ~USART_CR2_STOP_0;
	USART2->CR2 &= ~USART_CR2_STOP_1;

	USART2->CR1 |= USART_CR1_UE;
}

char usart_read(void){

	while(!(USART2->SR & USART_SR_RXNE));

	return USART2->DR;

}

void usart_write(int data){

	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = data & 0xFF;
	while(!(USART2->SR & USART_SR_TC));
}

void usart_set_baudrate(USART_TypeDef *usart, uint32_t peripheral_clock, uint32_t baudrate){

	usart->BRR = ((peripheral_clock + (baudrate/2U)) / baudrate);
}

void usart_general_init(USART_TypeDef *usart){

	if(usart == USART1){

		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	}else if(usart == USART2){

		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	}else if(usart == USART3){

		RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;

	}else if(usart == USART6){

		RCC -> APB2ENR |= RCC_APB2ENR_USART6EN;

	}

	usart_set_baudrate(usart, PERIPHERAL_CLOCK, BAUDRATE);

	usart->CR1 |= USART_CR1_TE;
	usart->CR1 |= USART_CR1_RE;

	//Set M = 0 for 8 bit packets
	usart->CR1 &= ~USART_CR1_M;
	//1 Stop bit
	usart->CR2 &= ~USART_CR2_STOP_0;
	usart->CR2 &= ~USART_CR2_STOP_1;

	usart->CR1 |= USART_CR1_UE;
}

char usart_general_read(USART_TypeDef *usart){

	while(!(usart->SR & USART_SR_RXNE));

	return usart->DR;
}

void usart_general_write(USART_TypeDef *usart, int data){

	while(!(usart->SR & USART_SR_TXE));
	usart->DR = data & 0xFF;
	while(!(usart->SR & USART_SR_TC));
}
