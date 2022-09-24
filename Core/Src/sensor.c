/*
 * sensor.c
 *
 *  Created on: 2022/09/24
 *      Author: kawauchikentarou
 */

#include "sensor.h"

uint16_t ADC1_Buff[ADC_DATA_BUFFR_SIZE];
uint16_t line_sen0;
uint16_t line_sen1;
uint16_t line_sen2;
uint16_t line_sen3;
uint16_t line_sen4;
uint16_t line_sen5;
uint16_t line_sen6;
uint16_t line_sen7;
uint16_t line_sen8;
uint16_t line_sen9;
uint16_t line_sen10;
uint16_t line_sen11;
uint16_t line_sen12;
uint16_t line_sen13;
uint16_t line_senLLL = 0;
uint16_t line_senLL = 0;
uint16_t line_senL = 0;
uint16_t line_senR = 0;
uint16_t line_senRR = 0;
uint16_t line_senRRR = 0;
int16_t enc_tim1_ms = 0;
int16_t enc_tim8_ms = 0;
int64_t enc_tim1_total = 0;
int64_t enc_tim8_total = 0;
int64_t enc_tim_total = 0;
int32_t enc_tim1_cnt_10ms;
int32_t enc_tim8_cnt_10ms;
int64_t enc_cnt;
int64_t enc_cnt2;
float velR, monR;
float velL, monL;
int posR;
int posL;
char error_flag = 0;
uint16_t error_cnt;
int timer = 0;
unsigned char main_pattern = 0;
uint8_t maker_check;
char crossline_flag = 0;
unsigned char velocity_pattern = 0;
int encoder_event = 0;
uint8_t flash_flag = 0;
uint32_t log_adress;
uint32_t* flash_read_test;
uint16_t maker_cnt;
uint16_t maker_flag;
float log_mm, log_zg;
float PlanVelo[6000];
int16_t log_array = 0;
uint32_t plan_velo_adress;
uint8_t second_trace_flag = 0;
float mm_total = 0;
uint32_t maker_adress;
uint16_t maker_distance_cmp_lim;
float PlanVelo2[6000];
uint8_t second_trace_pattern;

void ADval_get(void) {
	line_sen0  = ADC1_Buff[0];
	line_sen1  = ADC1_Buff[1];
	line_sen2  = ADC1_Buff[2];
	line_sen3  = ADC1_Buff[3];
	line_sen4  = ADC1_Buff[4];
	line_sen5  = ADC1_Buff[5];
	line_sen6  = ADC1_Buff[6];
	line_sen7  = ADC1_Buff[7];
	line_sen10 = ADC1_Buff[8];
	line_sen11 = ADC1_Buff[9];
	line_sen8  = ADC1_Buff[10];
	line_sen9  = ADC1_Buff[11];
	line_sen12 = ADC1_Buff[12];
	line_sen13 = ADC1_Buff[13];
}

void ADval_sum(void) {
	line_senLLL	= line_sen11 + line_sen10;
	line_senLL	= line_sen9 + line_sen8;
	line_senL	= line_sen7 + line_sen6;
	line_senR	= line_sen5 + line_sen4;
	line_senRR	= line_sen3 + line_sen2;
	line_senRRR	= line_sen1 + line_sen0;
}

uint8_t MakerSenTh(uint16_t makerthreshold) {
	uint8_t maker = 0;

	if(crossline_flag == 0){
		if(line_sen12 < makerthreshold) maker |= 0x01;
		if(line_sen13 < makerthreshold) maker |= 0x08;
	}

	return maker;
}


void getEncoder(void) {

	int16_t enc_tim1_ms;
	int16_t enc_tim8_ms;

	enc_tim1_ms = TIM1 -> CNT;
	enc_tim8_ms = TIM8 -> CNT;

	TIM1 -> CNT = 0;
	TIM8 -> CNT = 0;

	enc_tim1_total += enc_tim1_ms;
	enc_tim8_total += enc_tim8_ms;
	enc_tim_total = (enc_tim1_total + enc_tim8_total) / 2;

	enc_cnt += ((enc_tim1_ms + enc_tim8_ms) / 2.0f);
	enc_cnt2 += ((enc_tim1_ms + enc_tim8_ms) / 2.0f);

	enc_tim1_cnt_10ms += enc_tim1_ms;
	enc_tim8_cnt_10ms += enc_tim8_ms;

	velR = -(float)enc_tim1_ms * ENC_PULSE_MM * 1000.0f;
	velL = (float)enc_tim8_ms * ENC_PULSE_MM * 1000.0f;
	monR = velR;
	monL = velL;

}
