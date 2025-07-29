/*
 * pwm.h
 *
 *  Created on: Jul 19, 2025
 *      Author: prate
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

void pwm_init(uint16_t duty, uint16_t period);
void pwm_general_out_init(TIM_TypeDef *TIMx, uint8_t channel, uint16_t prescaler, uint16_t period, uint16_t duty);
void pwm_update_duty_cycle(TIM_TypeDef *TIMx, uint8_t channel, uint16_t duty);
void pwm_update_frequency(TIM_TypeDef *TIMx, uint16_t prescaler);
void pwm_stop(TIM_TypeDef *TIMx);
void pwm_start(TIM_TypeDef *TIMx);

#endif /* INC_PWM_H_ */
