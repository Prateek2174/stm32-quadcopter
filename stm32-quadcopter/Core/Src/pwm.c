/*
 * pwm.c
 *
 *  Created on: Jul 19, 2025
 *      Author: prate
 */

#include "stm32f4xx.h"
#include "pwm.h"

void pwm_init(uint16_t duty, uint16_t period){

	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1 -> CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM1 -> CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM1 -> CCMR2 &= ~TIM_CCMR2_CC3S;
	TIM1 -> CCMR2 &= ~TIM_CCMR2_CC4S;

	TIM1 -> CCER &= ~TIM_CCER_CC1P;
	TIM1 -> CCER &= ~TIM_CCER_CC2P;
	TIM1 -> CCER &= ~TIM_CCER_CC3P;
	TIM1 -> CCER &= ~TIM_CCER_CC4P;

	TIM1 -> CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_2;

	TIM1 -> CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC2M_2;

	TIM1 -> CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM1 -> CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM1 -> CCMR2 |= TIM_CCMR2_OC3M_2;

	TIM1 -> CCMR2 &= ~TIM_CCMR2_OC4M;
	TIM1 -> CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM1 -> CCMR2 |= TIM_CCMR2_OC4M_2;

	TIM1 -> PSC = 15999; //PSC divides the 16MHz clock -> counter running at 1 kHz
	TIM1 -> ARR = period;

	if(duty > period) duty = period;

	TIM1 -> CCR1 = duty;
	TIM1 -> CCR2 = duty;
	TIM1 -> CCR3 = duty;
	TIM1 -> CCR4 = duty;

	TIM1 -> CCMR1 |= TIM_CCMR1_OC1PE;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC2PE;
	TIM1 -> CCMR2 |= TIM_CCMR2_OC3PE;
	TIM1 -> CCMR2 |= TIM_CCMR2_OC4PE;

	TIM1 -> CR1 |= TIM_CR1_ARPE;

	TIM1 -> CCER |= TIM_CCER_CC1E;
	TIM1 -> CCER |= TIM_CCER_CC2E;
	TIM1 -> CCER |= TIM_CCER_CC3E;
	TIM1 -> CCER |= TIM_CCER_CC4E;

	TIM1 -> CR1 |= TIM_CR1_CEN;

	TIM1 -> BDTR |= TIM_BDTR_MOE;
}

void pwm_general_out_init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler, uint16_t period, uint16_t duty){

	if(channel < 1 || channel > 4) return;

	if (TIMx == TIM2) RCC -> APB1ENR |= (1U << 0);
		else if(TIMx == TIM3) RCC -> APB1ENR |= (1U << 1);
		else if(TIMx == TIM4) RCC -> APB1ENR |= (1U << 2);
		else if(TIMx == TIM5) RCC -> APB1ENR |= (1U << 3);
		else return;

	if(channel == 1 || channel == 2){

		TIMx -> CCMR1 &= ~(0x3U << ((channel - 1)*8));

	}else if(channel == 3 || channel == 4){

		TIMx -> CCMR2 &= ~(0x3U << ((channel - 3)*8));
	}

	//sets active high only for now
	TIMx -> CCER &= ~(0x1U << (1U + (channel - 1U)*4));

	//Sets PWM MODE 1 for now only
	if(channel == 1 || channel == 2){

		TIMx -> CCMR1 |= (6U << ((channel - 1)*8 + 4));

	}else if(channel == 3 || channel == 4){

		TIMx -> CCMR2 |= (6U << ((channel - 3)*8 + 4));
	}

	//Program the period and duty cycle in ARR and CCRx
	TIMx -> PSC = prescaler;
	TIMx -> ARR = period;

	switch(channel){
		case 1: TIMx -> CCR1 = duty; break;
		case 2: TIMx -> CCR2 = duty; break;
		case 3: TIMx -> CCR3 = duty; break;
		case 4: TIMx -> CCR4 = duty; break;

	}

	//Set the preload bit in CCMRx register and the ARPE bit in the CR1 register
	if(channel == 1 || channel == 2){

		TIMx -> CCMR1 |= (1U << (3 + (channel - 1)*8));

	}else if(channel == 3 || channel == 4){

		TIMx -> CCMR2 |= (1U << (3 + (channel - 3)*8));
	}

	TIMx -> CR1 |= (1U << 7);

	//Select a) PWM edge aligned mode: counting mode (up or down counting) b) PWM center aligned mode: the counter mode must be center aligned counting mode
	//LEAVE AS DEFAULT DIR = 0 upcount and CMS = 00 edge triggered

	//Enable the capture compare
	TIMx -> CCER |= (1U << (4*(channel-1)));
	//Enable the counter -> done in pwm_start
//	TIMx -> CR1 |= (1U << 0);
}

void pwm_update_duty_cycle(TIM_TypeDef *TIMx, uint8_t channel, uint16_t duty){

	if(channel < 1 || channel > 4) return;

	switch(channel){
			case 1: TIMx -> CCR1 = duty; break;
			case 2: TIMx -> CCR2 = duty; break;
			case 3: TIMx -> CCR3 = duty; break;
			case 4: TIMx -> CCR4 = duty; break;

	}
}

void pwm_update_frequency(TIM_TypeDef *TIMx, uint16_t prescaler){

	TIMx -> PSC = prescaler;
}

void pwm_stop(TIM_TypeDef *TIMx){

	TIMx -> CR1 &= ~(1U << 0);
}

void pwm_start(TIM_TypeDef *TIMx){

	TIMx -> CR1 |= (1U << 0);
}
