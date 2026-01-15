/*
 * handler_adc.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */

#ifndef INC_HANDLER_ADC_H_
#define INC_HANDLER_ADC_H_
#include <stdint.h>
#include "main.h"

#define ADC_CH_NUM        	4
#define ADC_SAMPLE_COUNT   	200
#define SKIP_VALUE			5
#define SKIP_PERCENT		3
#define DETECT_VALUE 		1500

typedef struct adc_handler{
//	uint16_t adc_in0[ADC_SAMPLE_COUNT];
	uint16_t adc_in1[ADC_SAMPLE_COUNT];
	uint16_t adc_in2[ADC_SAMPLE_COUNT];
//	uint16_t adc_in3[ADC_SAMPLE_COUNT];
}adc_caculate_t;

typedef enum{
	REAL_PR_OK =0,
	REAL_PR_NG,
	NO_PRODUCT,
	FIRT_SAMPLE,
	WRONG_SAMPLE
}REAL_VALUE;
typedef struct
{
    uint16_t ok;
    uint16_t ng;
} SampleValue_t;

typedef struct
{
    SampleValue_t left;
    SampleValue_t right;
//    uint16_t average_left;
//    uint16_t average_right;
} Sammple_t;


void ADC1_Calibration(void);
int compare(const void *a, const void *b);
uint16_t Average_Caculate(uint16_t * adc_array, uint8_t size, uint8_t skip);
void quickSortArray_uint16(uint16_t arr[], int n);
uint16_t MedianAverage(uint16_t *adc_array, uint8_t size);

#endif /* INC_HANDLER_ADC_H_ */
