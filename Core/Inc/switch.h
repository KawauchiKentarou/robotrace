/*
 * switch.h
 *
 *  Created on: Sep 23, 2022
 *      Author: kawauchikentarou
 */


#include "main.h"
#include "AQM0802.h"
#include "ICM20648.h"
#include "INA260.h"
#include "Flash_F405.h"


#ifndef INC_SWITCH_H_
#define INC_SWITCH_H_
#define CHECK_SENS_MAX	14

extern int8_t setup_mode;
extern int8_t check_sens_val;
extern uint8_t sw_up_state;
extern uint8_t sw_center_state;
extern uint8_t cnt_sw;


void HAL_GPIO_EXTI_Callback(uint16_t);

#endif /* INC_SWITCH_H_ */
