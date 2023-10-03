/*
 * adc_function.h
 *
 *  Created on: 27 sep. 2023
 *      Author: nicol
 */

#ifndef INC_ADC_FUNCTION_H_
#define INC_ADC_FUNCTION_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define ADC_OS				OS_START_1CONVERSION
#define AIN_VOLTAGE			AIN_23
#define AIN_CURRENT			AIN_01
#define ADC_PGA				FSR_2048
#define ADC_MODE			MODE_SINGLE
#define ADC_SPS				DR_860
#define ADC_COMP_MODE		COMP_TRADITIONAL
#define ADC_COMP_POL		COMP_LO
#define ADC_COMP_LAT		COMP_NON_LATCH
#define ADC_COMP_QUE		COMP_1

#define SCALE_CORRIENTE 0.457763 //mA por cuenta
#define SCALE_TENSION 2.4414 //mV por cuenta


enum input_to_measure {
	tension,
	corriente
};

float ADC_read_tension (I2C_HandleTypeDef *hi2c); //return mV

float ADC_read_corriente (I2C_HandleTypeDef *hi2c); //return mA

#endif /* INC_ADC_FUNCTION_H_ */
