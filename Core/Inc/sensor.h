/*
 * sensor.h
 *
 *  Created on: Sep 23, 2022
 *      Author: kawauchikentarou
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
/*#include "AQM0802.h"
#include "ICM20648.h"
#include "INA260.h"
#include "Flash_F405.h"
#include "sensor.h"
#include "switch.h"*/
#define ADC_DATA_BUFFR_SIZE		((uint16_t)14)
#define SENSOR_NUMBER 13
#define ENC_PULSE_MM 	0.012914f/*0.012207f*/		//0.024414f 2逓倍//0.025829
// 速度 = 1msでカウントしたパルス　* 1パルスで進む距離 * 1000 [mm/s]
// 1msで数mm進むのでm/s これに1000かけてmm/s
// 1pulseで進む距離　= タイヤ周長 / (エンコーダのパルス * n逓倍 * 減速比)
//								= 68mm / (512 * 4 * 2.72) = 0.0122...
//								= 72mm / (512 * 4 * 2.72) = 0.012914...

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart1;

uint16_t ADC1_Buff[ADC_DATA_BUFFR_SIZE];
uint8_t MakerSenTh(uint16_t);
extern uint16_t line_sen0;
extern uint16_t line_sen1;
extern uint16_t line_sen2;
extern uint16_t line_sen3;
extern uint16_t line_sen4;
extern uint16_t line_sen5;
extern uint16_t line_sen6;
extern uint16_t line_sen7;
extern uint16_t line_sen8;
extern uint16_t line_sen9;
extern uint16_t line_sen10;
extern uint16_t line_sen11;
extern uint16_t line_sen12;
extern uint16_t line_sen13;
extern uint16_t line_senLLL;
extern uint16_t line_senLL;
extern uint16_t line_senL;
extern uint16_t line_senR;
extern uint16_t line_senRR;
extern uint16_t line_senRRR;
extern int64_t enc_tim1_total;
extern int64_t enc_tim8_total;
extern int64_t enc_tim_total;
extern int64_t enc_tim_ML;
extern int64_t enc_tim_MR;
extern int64_t enc_tim_MC;
extern int32_t enc_tim1_cnt_10ms;
extern int32_t enc_tim8_cnt_10ms;
extern int64_t enc_cnt;
extern int64_t enc_cnt2;
extern char error_flag;
extern uint16_t error_cnt;
extern int timer;
extern float target_vel;
extern unsigned char main_pattern;
extern uint8_t maker_check;
extern int crossline_flag_L;
extern int crossline_flag_M;
extern int L_Maker_flag;
extern int R_Maker_flag;
extern unsigned char velocity_pattern;
extern int encoder_event;
extern uint8_t flash_flag;
extern uint32_t log_adress;
extern uint32_t* flash_read_test;
extern uint16_t maker_cnt;
extern int16_t log_array;
extern uint32_t plan_velo_adress;
extern uint8_t second_trace_flag;
extern float mm_total;
extern uint32_t maker_adress;
extern float maker_distance_L[];
extern float maker_distance_R[];
extern uint16_t maker_distance_cmp_lim;
extern float log_zg;
extern float log_mm;
extern float PlanVelo[];
extern float PlanVelo2[];
extern uint8_t second_trace_pattern;
//extern int8_t setup_mode;
extern int8_t check_sens_val;
extern uint8_t sw_up_state;
extern uint8_t sw_center_state;
extern uint8_t cnt_sw;
extern uint16_t analog[SENSOR_NUMBER];
extern uint16_t ADC_dif[SENSOR_NUMBER];
extern uint16_t ADC_max[SENSOR_NUMBER];
extern uint16_t ADC_min[SENSOR_NUMBER];
extern uint8_t MakerSenTh(uint16_t);
extern uint8_t CrossCheck(uint16_t);
extern uint8_t R_cnt;
extern uint8_t L_cnt;
extern uint8_t MR_cn;
extern uint8_t ML_cnt;
extern uint8_t CR_cnt;
extern int R_flag;
extern int L_flag;
extern int MR_flag;
extern int ML_flag;
extern int GL_flag;
extern unsigned char maker_pattern;
extern uint8_t Maker_senL;
extern uint8_t Maker_senR;
extern uint8_t Sensor_st;


void peripheral_init(void);
void gpio_set(void);
void led_pattern(uint8_t);
void getEncoder(void);
void ADval_get(void);
void ADval_sum(void);
void ADC_init(void);
void Cross_Check(void);
void Maker_Check(void);

#endif /* INC_SENSOR_H_ */
