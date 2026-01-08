#include "dvr_gpio.h"
#include "drv_adc.h"
#include "mgr.h"
uint32_t ADC_good=0x00U;
uint32_t ADC_Ngood=0x00U;
uint32_t ADC_TB=0x00U;
void Task_Ir_Run()
{
	if(Get_State_Sensor(0)==0x00)// button 1 -> sản phẩm tốt
	{
		// lấy giá trị adc 1 và adc2
		ADC_good=Adc_getvalue(0)+Adc_getvalue(1);
		ADC_good = ADC_good/2;
		ADC_TB =(ADC_good+ADC_Ngood)/2;
		//lưu vào memory
	}
	if(Get_State_Sensor(1)==0x00)// button 1 -> sản phẩm tốt
	{
		// lấy giá trị adc 1 và adc2
		ADC_Ngood=Adc_getvalue(0)+Adc_getvalue(1);
		ADC_Ngood = ADC_Ngood/2;
		ADC_TB =(ADC_good+ADC_Ngood)/2;
		//lưu vào memory
	}
	if(Get_State_Sensor(0)<ADC_TB && Get_State_Sensor(1)<ADC_TB)
	{
		// san pham good-> sang led
	}
	else
	{
		// san pham not good -> tat led
	}


}
