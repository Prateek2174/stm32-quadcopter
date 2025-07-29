/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "pwm.h"
#include "gpio.h"
#include "mpu6050.h"

#define PERIOD 100
#define DUTY 50

int main(void)
{
	//Use -> USART2

	//pwm test code
	//Enable clocks for GPIOA and TIM2
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;

	//Set alternative function PA5
	GPIOA -> MODER |= GPIO_MODER_MODE5_1;
	GPIOA -> AFR[0] |= GPIO_AFRL_AFRL5_0;

	//Set CC1 channel to output mode
	TIM2 -> CCMR1 &= ~TIM_CCMR1_CC1S;
	//Select the polarity active high or active low
	TIM2 -> CCER &= ~TIM_CCER_CC1P;

	//Select the PWM mode
	TIM2 -> CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM2 -> CCMR1 |= TIM_CCMR1_OC1M_2;

	//Program the period and duty cycle in ARR and CCRx
	TIM2 -> PSC = 15999; //PSC divides the 16MHz clock -> counter running at 1 kHz
	TIM2 -> ARR = PERIOD;
	TIM2 -> CCR1 = DUTY;

	//Set the preload bit in CCMRx register and the ARPE bit in the CR1 register
	TIM2 -> CCMR1 |= TIM_CCMR1_OC1PE;
	TIM2 -> CR1 |= TIM_CR1_ARPE;

	//Select a) PWM edge aligned mode: counting mode (up or down counting) b) PWM center aligned mode: the counter mode must be center aligned counting mode
	//LEAVE AS DEFAULT DIR = 0 upcount and CMS = 00 edge triggered

	//Enable the capture compare
	TIM2 -> CCER |= TIM_CCER_CC1E;

	//Enable the counter
	TIM2 -> CR1 |= TIM_CR1_CEN;


	while(1){

//		float pitch_output = pid_compute(&pitch_pid, desired_pitch, measured_pitch);
//		float roll_output  = pid_compute(&roll_pid, desired_roll, measured_roll);
//		float yaw_output   = pid_compute(&yaw_pid, desired_yaw, measured_yaw);
//
//		motor1 = throttle + pitch_output + roll_output - yaw_output; // Front-left
//		motor2 = throttle + pitch_output - roll_output + yaw_output; // Front-right
//		motor3 = throttle - pitch_output - roll_output - yaw_output; // Rear-right
//		motor4 = throttle - pitch_output + roll_output + yaw_output; // Rear-left
//
//		pwm_update_duty_cycle(TIM1, 1, clamp(motor1));
//		pwm_update_duty_cycle(TIM1, 2, clamp(motor2));
//		pwm_update_duty_cycle(TIM1, 3, clamp(motor3));
//		pwm_update_duty_cycle(TIM1, 4, clamp(motor4));

	}
}

