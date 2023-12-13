/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/

/* Private includes ----------------------------------------------------------*/
#include "adc_function.h"
#include "string.h"
#include "main.h"

#ifdef ADC_HW
	#if ADC_HW == ADS1115
		#include "ads1115.h"
		#define ADC_ADDR ADS1115_ADDR
	#endif
#endif

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variable ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void ADC_set (I2C_HandleTypeDef *hi2c, MEASURE_HandleTypeDef *hmeasure);

float ADC_read_tension (I2C_HandleTypeDef *hi2c, MEASURE_HandleTypeDef *hmeasure){

	uint8_t buffer[2];
	uint16_t buffer16;

	hmeasure->input = TENSION;

	ADC_set(hi2c, hmeasure);

	HAL_I2C_Master_Receive(hi2c, ADC_ADDR, buffer, 2, HAL_MAX_DELAY);

	buffer16 = buffer[0] << 8 | buffer[1];

	return buffer16*SCALE_TENSION+80000; //Devuelvo mV //Sumo la mitad por diferencial
}

float ADC_read_corriente (I2C_HandleTypeDef *hi2c, MEASURE_HandleTypeDef *hmeasure){

	uint8_t buffer[2];
	int16_t buffer16;

	hmeasure->input = CORRIENTE;

	ADC_set(hi2c, hmeasure);

	HAL_I2C_Master_Receive(hi2c, ADC_ADDR, buffer, 2, HAL_MAX_DELAY);

	buffer16 = buffer[0] << 8 | buffer[1];

	return buffer16 * SCALE_CORRIENTE * (hmeasure->pga) + OFFSET_CORRIENTE; //Devuelvo mA //Sumo la mitad por diferencial
}

/* Setea la configuracion del ADC segun la estructura hmeasure en modo: 1 lectura */

static void ADC_set (I2C_HandleTypeDef *hi2c, MEASURE_HandleTypeDef *hmeasure){

	uint8_t buffer[3],sps,pga;
	uint16_t buffer16;

	/* Para inicializacion del pga ADC */
	switch (hmeasure->pga){

	case FSR0256:
		pga = FSR_0256;
		break;

	case FSR0512:
		pga = FSR_0512;
		break;

	case FSR1024:
		pga = FSR_1024;
		break;

	case FSR2048:
		pga = FSR_2048;
		break;

	case FSR4096:
		pga = FSR_4096;
		break;

	case FSR6144:
	default:
		pga = FSR_6144;
		break;
	}

	/* Para inicializacion del sps ADC */
	switch (hmeasure->ratio){

	case SPS8:
		sps = DR_8;
		break;

	case SPS16:
		sps = DR_16;
		break;

	case SPS32:
		sps = DR_32;
		break;

	case SPS64:
		sps = DR_64;
		break;

	case SPS128:
		sps = DR_128;
		break;

	case SPS250:
		sps = DR_250;
		break;

	case SPS475:
		sps = DR_475;
		break;

	case SPS860:
	default:
		sps = DR_860;
		break;
	}

	/* Para el registro de configuracion ADC */
	buffer[0] = (uint8_t)CONFIG_REGISTER;
	if( hmeasure->input == TENSION){
		buffer16 = GET_CONFIG_REGISTER(ADC_OS,AIN_VOLTAGE,pga,ADC_MODE,sps,ADC_COMP_MODE,ADC_COMP_POL,ADC_COMP_LAT,ADC_COMP_QUE);
	}
	else if( hmeasure->input == CORRIENTE ){
		buffer16 = GET_CONFIG_REGISTER(ADC_OS,AIN_CURRENT,pga,ADC_MODE,sps,ADC_COMP_MODE,ADC_COMP_POL,ADC_COMP_LAT,ADC_COMP_QUE);
	}

	buffer[1] = (uint8_t)(buffer16 >> 8);
	buffer[2] = (uint8_t)(buffer16);

	HAL_I2C_Master_Transmit(hi2c, ADC_ADDR, buffer, 3, HAL_MAX_DELAY);

	buffer[0] = (uint8_t)CONVERSION_REGISTER;
	HAL_I2C_Master_Transmit(hi2c, ADC_ADDR, buffer, 1, HAL_MAX_DELAY);
}
