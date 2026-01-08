/*
 * drv_adc.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Admin
 */

#ifndef INC_DRV_ADC_H_
#define INC_DRV_ADC_H_

#define NUMBER_OF_CHANEL_ADC 0x04
#define NUMBER_OF_SAMPLING   30U

#include "main.h"
void ADC_Init_main(void);
void ADC_run(void);
uint32_t Adc_getvalue(uint8_t chanel);
void ADC1_Calibration(void);

#endif /* INC_DRV_ADC_H_ */
