/*
 * pid.h
 *
 *  Created on: Jul 23, 2025
 *      Author: prate
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

typedef struct{

	float kp;
	float ki;
	float kd;

	//Derivative low-pass filter time constant
	float tau;

	//Sample Time
	float Ts;

	//Output Limits
	float max_output_signal;
	float min_output_signal;

	//Controller Memory
	float integrator;
	float prev_error;
	float differentiator;
	float prev_measurement;

	//Controller Output
	float output;

}PID_controller;

void pid_init(PID_controller *pid, float kp, float ki, float kd, float tau, float Ts, float min, float max);
float pid_update(PID_controller *pid, float setpoint, float measurement);

#endif /* SRC_PID_H_ */
