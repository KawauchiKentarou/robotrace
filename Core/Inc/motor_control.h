/*
 * motor_control.h
 *
 *  Created on: Sep 23, 2022
 *      Author: kawauchikentarou
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"
#include "AQM0802.h"
#include "ICM20648.h"
#include "INA260.h"
#include "Flash_F405.h"
#include "sensor.h"
#include "switch.h"

#define DELTA_T 0.001f

extern float velR;
extern float velL;
extern int posR;
extern int posL;
/*
extern float target_vel;
extern unsigned char velocity_pattern;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
*/

extern float order_posR;
extern float order_posL;
extern float order_velR;
extern float order_velL;
extern float target_vel;
extern float mon_def_pos;
extern float motor_pwmL;
extern float motor_pwmR;
extern int i_vel_clear_flag;
extern int i_pos_clear_flag;


#define MR_SET 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define MR_RESET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define ML_SET 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define ML_RESET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

#define COUNTER_PERIOD_TIM12	839

#define ESC_MAX 		3527	//84[us]
#define ESC_MIN			1763	//42[us]

void gpio_set(void);
void Motorset(int16_t, int16_t, uint8_t);
void posPID();
void velPID(float);


#endif /* INC_MOTOR_CONTROL_H_ */
