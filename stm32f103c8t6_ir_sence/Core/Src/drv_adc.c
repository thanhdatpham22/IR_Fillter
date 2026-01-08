


#include "drv_adc.h"
extern ADC_HandleTypeDef hadc1;
volatile static uint16_t a[4];
volatile static uint32_t adc_avg[NUMBER_OF_CHANEL_ADC]={};
void ADC_Init_main(void)
{
	 HAL_ADC_Start_DMA(&hadc1,(uint32_t *)a,4);
}

void ADC_run(void)
{
	volatile static uint32_t sum_of_sampling[NUMBER_OF_CHANEL_ADC]={};
	static uint8_t count=0x00U;
	if(DMA1->ISR &(1<<1U))
	{
		for(int i=0;i<NUMBER_OF_CHANEL_ADC;i++)
		{
			sum_of_sampling[i] +=(uint32_t)a[i];
		}
		if(++count>=NUMBER_OF_SAMPLING)
		{
			count=0x00U;
			for(int i=0;i<NUMBER_OF_CHANEL_ADC;i++)
			{
				adc_avg[i] = (sum_of_sampling[i]/NUMBER_OF_SAMPLING);
				sum_of_sampling[i]=0x00U;
			}
		}
	}

}
uint32_t Adc_getvalue(uint8_t chanel)
{
	return adc_avg[chanel];
}
void ADC1_Calibration(void)
{
    // Bước 1: Đảm bảo ADC đã được kích hoạt Clock (Phải được thực hiện ở thanh ghi RCC trước đó)
    // Giả định: Clock cho ADC1 đã được bật (RCC_APB2ENR |= RCC_APB2ENR_ADC1EN)

    // Bước 2: Bật ADC (Thiết lập bit ADON)
    // Cần bật ADC trước khi hiệu chuẩn.
    ADC1->CR2 |= ADC_CR2_ADON;
.0
    // Chờ một thời gian ngắn (tối thiểu 2 chu kỳ clock ADC) để ADC ổn định (thao tác này thường đã đủ).

    // Bước 3: Thiết lập bit RSTCAL để reset giá trị hiệu chuẩn trước đó
    ADC1->CR2 |= ADC_CR2_RSTCAL;

    // Bước 4: Chờ cho bit RSTCAL tự động xóa (báo hiệu việc reset đã hoàn tất)
    // Vòng lặp chờ: Đợi cho thanh ghi tự động xóa bit RSTCAL
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0);

    // Bước 5: Thiết lập bit CAL để bắt đầu quá trình tự hiệu chuẩn
    ADC1->CR2 |= ADC_CR2_CAL;

    // Bước 6: Chờ cho bit CAL tự động xóa (báo hiệu quá trình hiệu chuẩn đã hoàn tất)
    // Vòng lặp chờ: Đợi cho thanh ghi tự động xóa bit CAL
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0);

    // Quá trình hiệu chuẩn ADC1 đã hoàn thành.
    // Mã hiệu chỉnh đã được tính toán và lưu nội bộ.
}
