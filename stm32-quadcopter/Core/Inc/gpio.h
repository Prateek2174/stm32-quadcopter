/*
 * gpio.h
 *
 *  Created on: Jul 19, 2025
 *      Author: prate
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f4xx.h"

//Function Prototypes
void gpio_init(void);
void gpio_general_init(GPIO_TypeDef *port, uint8_t pin, uint8_t mode, uint8_t af_num);

#endif /* INC_GPIO_H_ */
