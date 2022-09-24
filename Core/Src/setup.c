/*
 * setup.c
 *
 *  Created on: Sep 24, 2022
 *      Author: kawauchikentarou
 */
#include "setup.h"

uint32_t log_check_adress;

void setup(void){
	unsigned short volt_reg;

	//led_pattern(setup_mode);

	switch(setup_mode) {
		case 0:	//sensor check

		if( sw_center_state == 1 ) {	//buzzer
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1049); //MAX4199
		}
		else __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

		switch(check_sens_val) {
			case 0:
				lcd_locate(0,0);
				lcd_printf("%4d AD0",line_sen0);
				lcd_locate(0,1);
				lcd_printf("%4d AD1",line_sen1);
				break;
			case 1:
				lcd_locate(0,0);
				lcd_printf("%4d AD2",line_sen2);
				lcd_locate(0,1);
				lcd_printf("%4d AD3",line_sen3);
				break;
			case 2:
				lcd_locate(0,0);
				lcd_printf("%4d AD4",line_sen4);
				lcd_locate(0,1);
				lcd_printf("%4d AD5",line_sen5);
				break;
			case 3:
				lcd_locate(0,0);
				lcd_printf("%4d AD6",line_sen6);
				lcd_locate(0,1);
				lcd_printf("%4d AD7",line_sen7);
				break;
			case 4:
				lcd_locate(0,0);
				lcd_printf("%4d AD8",line_sen8);
				lcd_locate(0,1);
				lcd_printf("%4d AD9",line_sen9);
				break;
			case 5:
				lcd_locate(0,0);
				lcd_printf("%4dAD10",line_sen10);
				lcd_locate(0,1);
				lcd_printf("%4dAD11",line_sen11);
				break;
			case 6:
				lcd_locate(0,0);
				lcd_printf("%4dAD10",line_sen12);
				lcd_locate(0,1);
				lcd_printf("%4dAD11",line_sen13);
				break;
			case 7:
				lcd_locate(0,0);
				lcd_printf("XG%6x",xg);
				lcd_locate(0,1);
				lcd_printf("YG%6x",yg);
				break;
			case 8:
				lcd_locate(0,0);
				lcd_printf("ZG%6x",zg);
				lcd_locate(0,1);
				lcd_printf("XA%6x",xa);
				break;
			case 9:
				lcd_locate(0,0);
				lcd_printf("YA%6x",ya);
				lcd_locate(0,1);
				lcd_printf("ZA%6x",za);
				break;
			case 10:
				lcd_locate(0,0);
				lcd_print("Encoder1");
				lcd_locate(0,1);
				//lcd_printf("%8d", (int)mileage((float)enc_tim1_total));
				break;
			case 11:
				lcd_locate(0,0);
				lcd_print("Encoder2");
				lcd_locate(0,1);
				//lcd_printf("%8d", (int)mileage((float)enc_tim8_total));
				break;
			case 12:
				lcd_locate(0,0);
				lcd_print("Voltage_");
				lcd_locate(0,1);
				volt_reg = INA260_read(0x02);
				lcd_printf("   %1.2fV",(float)volt_reg*0.00125f);
				break;
			case 13:
				lcd_locate(0,0);
				lcd_print("error_th");
				lcd_locate(0,1);
				lcd_printf("%8d",line_senLLL + line_senLL + line_senL + line_senR + line_senRR + line_senRRR);
				break;
			case 14:
				lcd_locate(0,0);
				lcd_print("cross_th");
				lcd_locate(0,1);
				lcd_printf("%8d", line_senLL + line_senL + line_senR + line_senRR);
				break;
			default:
				break;
			}

			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

			lcd_locate(0,0);
			lcd_print("test_ESC");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			if( sw_center_state == 1 ) {

				/*
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2116);	//	1763(ESC_MIN) + 17.64 * 20
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2116);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2116);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2116);
				*/
			}
			else {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
			}
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);

			lcd_locate(0,0);
			lcd_print("test_MD_");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			if( sw_center_state == 1 ) {
				Motorset(400, 200, 0);
				/*
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 400);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 400);
				MR_SET;
				ML_SET;
				*/
			}
			else {
				Motorset(0, 0, 0);
				/*
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				MR_SET;
				ML_SET;
				*/
			}
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

			lcd_locate(0,0);
			lcd_print("_erase__");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			if( sw_center_state == 1 ) {
				if( isnan( *(float*)start_adress_sector7 ) == 0 )  FLASH_EreaseSector(FLASH_SECTOR_7);
				if( isnan( *(float*)start_adress_sector9 ) == 0 )  FLASH_EreaseSector(FLASH_SECTOR_9);
				if( isnan( *(float*)start_adress_sector10 ) == 0 ) FLASH_EreaseSector(FLASH_SECTOR_10);
				if( isnan( *(float*)start_adress_sector11 ) == 0 ) FLASH_EreaseSector(FLASH_SECTOR_11);
			}

			break;
		case 4:
			lcd_locate(0,0);
			lcd_print("_case-4_");
			lcd_locate(0,1);
			lcd_print("________");
			break;
		case 5:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 1 ");
			if(sw_center_state == 1) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				sw_center_state = 0;
				velocity_pattern = 1;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		case 6:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 2 ");
			if(sw_center_state == 1) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				sw_center_state = 0;
				log_check_adress = start_adress_sector10;
				if( isnan( *(float*)log_check_adress ) == 0 ) {
					second_trace_flag = 1;
					second_trace_pattern = 1;
				}
				else velocity_pattern = 2;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		case 7:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 3 ");
			if(sw_center_state == 1) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				sw_center_state = 0;
				log_check_adress = start_adress_sector11;
				if( isnan( *(float*)log_check_adress ) == 0 ) {
					second_trace_flag = 1;
					second_trace_pattern = 2;
				}
				else velocity_pattern = 3;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		default:
			break;
	}
}
