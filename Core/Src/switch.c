/*
 * switch.c
 *
 *  Created on: Sep 23, 2022
 *      Author: kawauchikentarou
 */


#include "switch.h"
#include "main.h"

int8_t setup_mode = 0;
int8_t check_sens_val = 0;
uint8_t sw_up_state = 0;
uint8_t sw_center_state = 0;
uint8_t cnt_sw = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(cnt_sw >= 30) {
		if (GPIO_Pin == GPIO_PIN_0) ; 	//left

		if (GPIO_Pin == GPIO_PIN_1) { 	//up
			if(sw_up_state > 1) sw_up_state = 1;
			sw_up_state ^= 1;
			check_sens_val--;
			if(check_sens_val < 0)  check_sens_val = CHECK_SENS_MAX;
		}

		if (GPIO_Pin == GPIO_PIN_12) {	//push
			setup_mode++;
			if(setup_mode >= 8) setup_mode = 0;
		}

		if (GPIO_Pin == GPIO_PIN_13) {	//down
			check_sens_val++;
			if(check_sens_val > CHECK_SENS_MAX) check_sens_val = 0;
		}

		if (GPIO_Pin == GPIO_PIN_14) {	//center
			if(sw_center_state > 1) sw_center_state = 1;
			sw_center_state ^= 1;
		}

		if (GPIO_Pin == GPIO_PIN_15) ;	//right

		cnt_sw = 0;
	}
}

