/*
 * usart.h
 *
 *  Created on: Jul 26, 2025
 *      Author: prate
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f4xx.h"

void usart_init(void);
char usart_read(void);
void usart_write(int data);
void usart_set_baudrate(USART_TypeDef *usart, uint32_t peripheral_clock, uint32_t baudrate);

void usart_general_init(USART_TypeDef *usart);
char usart_general_read(USART_TypeDef *usart);
void usart_general_write(USART_TypeDef *usart, int data);

#endif /* INC_USART_H_ */
