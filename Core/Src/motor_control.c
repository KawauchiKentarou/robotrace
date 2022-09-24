/*
 * motor_control.c
 *
 *  Created on: Sep 23, 2022
 *      Author: kawauchikentarou
 */
#include "motor_control.h"

float velR, monR;
float velL, monL;
int posR;
int posL;
float order_posR = 0.0f;
float order_posL = 0.0f;
float order_velR = 0.0f;
float order_velL = 0.0f;
float target_vel;

void gpio_set(void){
	CS_SET;
	MR_SET;
	ML_SET;
}

void Motorset(int16_t motorL, int16_t motorR, uint8_t stop) {

	int16_t pwmL_out,pwmR_out;

	if(motorR >= 0) {
		pwmR_out = motorR;
		MR_SET;
	}
	else {
		pwmR_out = motorR * (-1);
		MR_RESET;
	}

	if(motorL >= 0) {
		pwmL_out = motorL;
		ML_RESET;
	}
	else {
		pwmL_out = motorL*(-1) ;
		ML_SET;
	}

	if(pwmR_out > COUNTER_PERIOD_TIM12) pwmR_out = 839;
	if(pwmL_out > COUNTER_PERIOD_TIM12) pwmL_out = 839;

	if(stop == 1) {
		pwmR_out = 0;
		pwmL_out = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwmR_out);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwmL_out);

}



void posPID(void) {

	float p_pos, d_pos;
	static float i_pos;
	float kp_pos = 0.15f, ki_pos = 0.00f/*0.004f*/, kd_pos = 0.00f/*0.008f*/;
	static float def_pos[] = {0.0f, 0.0f};

	//def_pos[0] = ( ((float)line_senLLL * 1.6f) + ((float)line_senLL * 1.25f) + (float)line_senL) - ((float)line_senR + ((float)line_senRR * 1.25f) + ((float)line_senRRR * 1.6f)); //1.25 1.6
	def_pos[0] = ( ((float)line_senLLL) + ((float)line_senLL ) + (float)line_senL) - ((float)line_senR + ((float)line_senRR ) + ((float)line_senRRR )); //1.25 1.6

	p_pos = kp_pos * def_pos[0]; //P制御
	i_pos += ki_pos * def_pos[0] * DELTA_T; //I制御
	d_pos = kd_pos * (def_pos[0] - def_pos[1]) / DELTA_T; //D制御

	order_posR = -p_pos + i_pos + d_pos;
	order_posL = (p_pos + i_pos + d_pos);

	def_pos[1] = def_pos[0];

}

void velPID(float target) {
	float p_vel, kp_vel = 2.80f/*2.8f*/, ki_vel = 50.0f;	//2.8 50
	//float vel_center, filter_vel_center, acceleration_imu;
	static float i_vel, def_vel, vel_center;

	vel_center = (velR + velL) / 2.0f;
	//acceleration_imu = (float)xa / 16384.0f;
	//filter_vel_center = ComplementaryFilter(acceleration_imu, vel_center, 0.65f, last_vel_center);
	//last_vel_center = filter_vel_center;

	def_vel = 300.0f - vel_center ;

	p_vel = kp_vel * def_vel;
	i_vel += ki_vel * def_vel * DELTA_T;

	order_velR = p_vel + i_vel;
	order_velL = p_vel + i_vel;
}

