/*
 * pid.c
 *
 *  Created on: Jul 23, 2025
 *      Author: prate
 */

#include "pid.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

void pid_init(PID_controller *pid, float kp, float ki, float kd, float tau, float Ts, float min, float max){

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->Ts = Ts;
	pid->tau = tau;
	pid->min_output_signal = min;
	pid->max_output_signal = max;


	pid -> integrator = 0.0f;
	pid -> prev_error = 0.0f;

	pid -> differentiator = 0.0f;
	pid -> prev_measurement = 0.0f;

	pid -> output = 0.0f;
}

float pid_update(PID_controller *pid, float setpoint, float measurement){

	//Error signal
	float error = setpoint - measurement;

	//Proportional
	float proportional = pid -> kp * error;

	//Integral
	pid -> integrator = pid -> integrator + 0.5f*(pid -> ki * pid -> Ts)*(error - pid -> prev_error);

	//Anti-wind-up with dynamic integrator clamping
	//clamp lower than physical limit
	float min_limit_integrator;
	float max_limit_integrator;

	//Compute integrator limits
	if(pid -> max_output_signal > proportional){

		max_limit_integrator = pid -> max_output_signal - proportional;

	}else{

		max_limit_integrator = 0.0f;
	}

	if(pid -> min_output_signal < proportional){

			min_limit_integrator = pid -> min_output_signal - proportional;
		}else{

			min_limit_integrator = 0.0f;
		}

	//Clamp Integrator
	if(pid -> integrator > max_limit_integrator){

		pid -> integrator = max_limit_integrator;

	}else if(pid -> integrator < min_limit_integrator){

		pid -> integrator = min_limit_integrator;
	}

	//Derivative (band-limited differentiator)
	pid -> differentiator = -(2.0f * pid->kd * (measurement - pid->prev_measurement) +
		    				(2.0f * pid->tau - pid->Ts) * pid->differentiator)
		    				/ (2.0f * pid->tau + pid->Ts);

	//Compute Output and Apply Limits
	pid -> output = proportional + pid -> integrator + pid -> differentiator;

	if(pid -> output > pid->max_output_signal){

		pid->output = pid->max_output_signal;

	}else if(pid->output < pid->min_output_signal){

		pid->output = pid->min_output_signal;
	}

	//Store error and measurement
	pid->prev_measurement = measurement;
	pid -> prev_error = error;

	return pid -> output;

}

