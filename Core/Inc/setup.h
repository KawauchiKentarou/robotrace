/*
 * setup.h
 *
 *  Created on: Sep 24, 2022
 *      Author: kawauchikentarou
 */

#ifndef INC_SETUP_H_
#define INC_SETUP_H_

#include "math.h"
#include "main.h"
#include "AQM0802.h"
#include "ICM20648.h"
#include "INA260.h"
#include "Flash_F405.h"
#include "sensor.h"
#include "switch.h"
#include "motor_control.h"

extern int8_t setup_mode;
extern int8_t sensor_mode;


void setup(void);
float mileage(float);

#endif /* INC_SETUP_H_ */
