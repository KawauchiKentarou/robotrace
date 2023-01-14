/*
 * sensor.c
 *
 *  Created on: 2022/09/24
 *      Author: kawauchikentarou
 */
#include "stdio.h"
#include "sensor.h"
#include "setup.h"

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
uint16_t Crossval = 0;
int16_t enc_tim1_ms = 0;
int16_t enc_tim8_ms = 0;
int64_t enc_tim1_total = 0;
int64_t enc_tim8_total = 0;
int64_t enc_tim_total = 0;
int64_t enc_tim_ML = 0;
int64_t enc_tim_MR = 0;
int64_t enc_tim_MC = 0;
int32_t enc_tim1_cnt_10ms;
int32_t enc_tim8_cnt_10ms;
int64_t enc_cnt;
int64_t enc_cnt2;
uint8_t Maker_senL = 0;
uint8_t Maker_senR = 0;
float velR, monR;
float velL, monL;
int posR;
int posL;
char error_flag = 0;
uint16_t error_cnt;
int timer = 0;
unsigned char main_pattern = 0;
uint8_t maker_pattern = 0;
uint8_t maker_check;
int crossline_flag_L = 0;
int crossline_flag_M = 0;
char crossline_flag = 0;
int L_Maker_flag = 0;
int R_Maker_flag = 0;
uint8_t R_cnt = 0;
uint8_t L_cnt = 0;
uint8_t MR_cnt = 0;
uint8_t ML_cnt = 0;
uint8_t CR_cnt = 0;
int R_flag = 0;
int L_flag = 0;
int MR_flag = 0;
int ML_flag = 0;
int GL_flag = 0;
uint8_t start_goal_flag = 0;

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
uint8_t MakerSenTh(uint16_t);
float PlanVelo2[6000];
uint8_t second_trace_pattern;
uint16_t ADC_min[SENSOR_NUMBER];
uint16_t ADC_max[SENSOR_NUMBER];
uint16_t ADC_dif[SENSOR_NUMBER];
uint8_t Sensor_st = 0;


void ADC_init(){
//	switch(sensor_mode) {
//	case 0:
		lcd_locate(0,0);
		lcd_print("genkini");
		lcd_locate(0,1);
		lcd_print("mokkori");
		HAL_Delay(100);
//		break;
}

/*	case 1:
		lcd_locate(0,0);
		lcd_print("genkini");
		lcd_locate(0,1);
		lcd_print("mokkori");
		break;

	}
}
	//Flash_load();
	//lcd_printf("ADCinit");
	HAL_Delay(100);
	uint16_t ADC_Max[SENSOR_NUMBER]={0};
	uint16_t i;

	for(int j=0;j<SENSOR_NUMBER;j++){
		ADC_min[j]=10000;
	}


	i = 0;
	while (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14))
	{
		if(analog[i] > ADC_Max[i]){
			ADC_Max[i] = analog[i];
		}
		if(analog[i] < ADC_min[i]){
			ADC_min[i] = analog[i];
		}
		i++;
		if(i == SENSOR_NUMBER){
			i=0;
		}
	//	LED(2);

	}
	for(int j=0;j<SENSOR_NUMBER;j++){
		ADC_dif[j] = ADC_max[j]-ADC_min[j];
	//	work_ram[z] = ADC_dif[z];
	//	work_ram[z+SENSOR_NUMBER] = ADC_Small[z];
	}
	//Flash_store();
	//for(int z=0;z<SENSOR_NUMBER;z++){
//		printf("%d,%d\r\n",z,ADC_Small[z]);
	//}
//	for(int z=0;z<SENSOR_NUMBER;z++){
//		printf("%d,%d\r\n",z,ADC_Max[z]);
//	}
//	LED(3);

}*/

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




void CrossCheck(uint16_t crossthreshold){

	Crossval = line_senLL + line_senL + line_senR + line_senRR;


	if(crossline_flag == 0 && line_senLL + line_senL + line_senR + line_senRR < crossthreshold ) {
		crossline_flag = 1;
		enc_cnt = 0;
	}

	if(crossline_flag == 1 && mileage((float)enc_cnt) >= 90){
		crossline_flag = 0;
	}
}

void Maker_Check(void) {


	if(line_sen12 >= 500 && line_sen13 >= 500){	//black
		Sensor_st = 0;
	}
	else if(line_sen12 <= 500 && line_sen13 <= 500){	//white
		Sensor_st = 3;
	}
	else if(line_sen12 <= 500){	//left
			Sensor_st = 1;
		}

	else if(line_sen13 <= 500){	//right
			Sensor_st = 2;
		}



	if(Sensor_st == 2){
		MR_cnt = 1;
	}
	else if(Sensor_st == 3){
		MR_cnt = 0;
	}

/*	if(MR_cnt >=5){
		maker_pattern = 2;

	}

	if(MR_cnt >=5  && maker_pattern == 0){
		MR_flag = 0;
		enc_cnt = 0;
		maker_pattern = 2;
	}*/


	if(MR_cnt == 1 && crossline_flag == 1){
			//maker_pattern = 0;
			MR_cnt= 0;
	}
	else if(MR_cnt == 1 && Sensor_st == 0 && crossline_flag == 0){
		MR_cnt = 0;
		GL_flag++;;
	}

	/*if(maker_pattern == 3 ){
		GL_flag++;
		maker_pattern = 0;
	}else{
		maker_pattern = 0;

	}*/


}
/*uint8_t MakerSenTh(uint16_t makerthreshold) {
	uint8_t maker = 0;

	if(crossline_flag == 0){
		if(line_sen12 < makerthreshold) maker |= 0x01;
		if(line_sen13 < makerthreshold) maker |= 0x08;
	}

	return maker;
}*/

uint8_t StartGoalCheck(uint8_t makerval) {
	uint8_t ret = 0;



/*	if( mileage((float)enc_cnt2) >= 20 && start_goal_flag == 0 && makerval == 8) {
		start_goal_flag = 1;
	}

	if( start_goal_flag == 1 ) {
		if(makerval == 0) {
			start_goal_flag = 0;
			ret = 1;
		}
		else if( (makerval &= 0x03) > 0 && (makerval &= 0x03) < 8) {
			start_goal_flag = 0;
			enc_cnt2 = 0;
		}
	}*/

	return ret;
}

void ADval_sum(void) {
	line_senLLL	= line_sen11 + line_sen10;
	line_senLL	= line_sen9 + line_sen8;
	line_senL	= line_sen7 + line_sen6;
	line_senR	= line_sen5 + line_sen4;
	line_senRR	= line_sen3 + line_sen2;
	line_senRRR	= line_sen1 + line_sen0;
}



void getEncoder(void) {

	int16_t enc_tim1_ms;
	int16_t enc_tim8_ms;

	enc_tim1_ms = TIM1 -> CNT;
	enc_tim8_ms = TIM8 -> CNT;

	TIM1 -> CNT = 0;
	TIM8 -> CNT = 0;

	enc_tim1_total -= enc_tim1_ms;
	enc_tim8_total += enc_tim8_ms;
	enc_tim_total = (enc_tim1_total + enc_tim8_total) / 2;

	enc_cnt += ((-enc_tim1_ms + enc_tim8_ms) / 2.0f);
	enc_cnt2 += ((-enc_tim1_ms + enc_tim8_ms) / 2.0f);

	enc_tim1_cnt_10ms += enc_tim1_ms;
	enc_tim8_cnt_10ms += enc_tim8_ms;

	velR = -(float)enc_tim1_ms * ENC_PULSE_MM * 1000.0f;
	velL = (float)enc_tim8_ms * ENC_PULSE_MM * 1000.0f;
	monR = velR;
	monL = velL;

}

float mileage(float mm) {
	return mm * ENC_PULSE_MM;
}

/*float ComplementaryFilter(float high_cut, float low_cut, float alpha, float complement_before) {
	float complement;

	complement = alpha * (complement_before + high_cut * DELTA_T) + (1.0f - alpha) * low_cut;

	return complement;
}*/

