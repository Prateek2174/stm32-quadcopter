/*
 * gpio.c
 *
 *  Created on: Jul 19, 2025
 *      Author: prate
 */
#include "gpio.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

#define GPIO_OSPEEDR_OSPEEDR6_0 (1U << 12)
#define GPIO_OSPEEDR_OSPEEDR6_1 (1U << 13)
#define GPIO_OSPEEDR_OSPEEDR7_0 (1U << 14)
#define GPIO_OSPEEDR_OSPEEDR7_1 (1U << 15)

void gpio_init(void){

	//PWM GPIO
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA -> MODER |= GPIO_MODER_MODE8_1;
	GPIOA -> AFR[1] |= GPIO_AFRH_AFSEL8_0;
	GPIOA -> MODER |= GPIO_MODER_MODE9_1;
	GPIOA -> AFR[1] |= GPIO_AFRH_AFSEL9_0;
	GPIOA -> MODER |= GPIO_MODER_MODE10_1;
	GPIOA -> AFR[1] |= GPIO_AFRH_AFSEL10_0;
	GPIOA -> MODER |= GPIO_MODER_MODE11_1;
	GPIOA -> AFR[1] |= GPIO_AFRH_AFSEL11_0;

	//I2C GPIO
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB -> MODER |= GPIO_MODER_MODE6_1;
	GPIOB -> MODER |= GPIO_MODER_MODE7_1;
	//Alternate function
	GPIOB -> AFR[0] |= GPIO_AFRL_AFRL6_2;
	GPIOB -> AFR[0] |= GPIO_AFRL_AFRL7_2;
	//Output open drain
	GPIOB -> OTYPER |= GPIO_OTYPER_OT6;
	GPIOB -> OTYPER |= GPIO_OTYPER_OT7;
	//Moderate speed
	GPIOB -> OSPEEDR |= GPIO_OSPEEDR_OSPEEDR6_0;
	GPIOB -> OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_1;
	GPIOB -> OSPEEDR |= GPIO_OSPEEDR_OSPEEDR7_0;
	GPIOB -> OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_1;

	//USART2 GPIO
	GPIOA->MODER |= GPIO_MODER_MODE2_1;
	GPIOA->MODER |= GPIO_MODER_MODE3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_2;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_2;

}

void gpio_general_init(GPIO_TypeDef *port, uint8_t pin, uint8_t mode, uint8_t af_num){

	if (port == GPIOA) RCC->AHB1ENR |= (1U << 0);
	else if (port == GPIOB) RCC->AHB1ENR |= (1U << 1);
    else if (port == GPIOC) RCC->AHB1ENR |= (1U << 2);
    else if (port == GPIOD) RCC->AHB1ENR |= (1U << 3);
    else if (port == GPIOE) RCC->AHB1ENR |= (1U << 4);
    else if (port == GPIOF) RCC->AHB1ENR |= (1U << 5);
    else if (port == GPIOG) RCC->AHB1ENR |= (1U << 6);
    else if (port == GPIOH) RCC->AHB1ENR |= (1U << 7);

	port -> MODER &= ~((3U) << (pin * 2)); //Clear any old bits
	port -> MODER |= ((mode & 3U) << (pin * 2));

	if(mode == 0x2U){

		if(pin < 8){

			port -> AFR[0] &= ~((0xF) << (pin * 4)); //Clear
			port -> AFR[0] |= ((af_num & 0xFU) << (pin * 4));

		}else if(pin > 7){

			port -> AFR[1] &= ~((0xFU) << ((pin - 8) * 4)); //Clear
			port -> AFR[1] |= ((af_num & 0xFU) << ((pin - 8) * 4));
		}
	}

}

//Seperate function that sets the appropriate alternate function: USART_init function will set the appropriate alternative function
