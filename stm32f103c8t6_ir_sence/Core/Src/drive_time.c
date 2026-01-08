/*
 * drive_time.c
 *
 *  Created on: Mar 2, 2025
 *      Author: Hi
 */
#include"drive_time.h"
#include"dvr_gpio.h"
Msg_Timer_Delay TID_Timer[TOTAL_TIMER_DELAY];
static volatile uint32_t _tGloabal_milis=0;
static volatile uint8_t timetick_100ms=0;
static volatile uint8_t giay60=0;

volatile uint8_t update_status=0x00U;

extern uint8_t count_lock;
extern uint32_t lastModbusTime;
uint32_t timedelayall[TOTAL_TIMER]={0,};
void Timer2_Init(void)
{
	// Enable Clock for Timer2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// Reset Timer
	TIM2->CR1 = 0;
	TIM2->PSC = 7;       // Prescaler (8MHz / (7 + 1) = 1MHz tick)
	TIM2->ARR = 999;      // Auto reload value (1ms)

	// Enable Update Interrupt
	TIM2->DIER |= TIM_DIER_UIE;

	// Enable Timer
	TIM2->CR1 |= TIM_CR1_CEN;

	// Enable NVIC for TIM2 IRQ
	NVIC_EnableIRQ(TIM2_IRQn);

}
uint32_t millis(void)
{
	return _tGloabal_milis;
}


void TIM2_IRQHandler(void)
{

    if (TIM2->SR & TIM_SR_UIF)  // Kiểm tra xem có ngắt xảy ra không
    {
      	++_tGloabal_milis;
      	Task_Gpio_input();
		if(_tGloabal_milis>=0x7FFFFFFFU)
		{
			_tGloabal_milis=0x00U;
			for(int i=0;i<TOTAL_TIMER_DELAY;i++)
			{
				TID_Timer[i].Time_Cur=0x00U;
				if(TID_Timer[i].End_Time>0x7FFFFFFFU)
				{
					TID_Timer[i].End_Time=TID_Timer[i].End_Time-0x7FFFFFFFU;
				}
				else
				{
					TID_Timer[i].End_Time=0x00U;
				}
			}
		}
		TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ UIF (bit 0)
    }
}
void reset_timer(void)
{

	for(int i=0;i<TOTAL_TIMER_DELAY;i++)
	{
		TID_Timer[i].active=0x00U;
		TID_Timer[i].Time_Delay=0x00U;
		TID_Timer[i].Time_Cur=0x00U;
		TID_Timer[i].End_Time=0x00U;
		_tGloabal_milis=0x00U;
	}

}
void reset_timer_one_channel(uint8_t id)
{
	TID_Timer[id].active=0x00U;
	TID_Timer[id].Time_Delay=0x00U;
	TID_Timer[id].Time_Cur=0x00U;
	TID_Timer[id].End_Time=0x00U;
}
void Delay_SetTimer(uint8_t id,uint32_t timer)
{
	TID_Timer[id].active=0x01U;
	TID_Timer[id].Time_Delay=timer;
	TID_Timer[id].Time_Cur=_tGloabal_milis;
	TID_Timer[id].End_Time=TID_Timer[id].Time_Cur+TID_Timer[id].Time_Delay;
}
uint8_t Delay_GetTimer(uint8_t id)
{
	TID_Timer[id].Time_Cur=_tGloabal_milis;
	if(TID_Timer[id].active==0x01U)
	{
		if(TID_Timer[id].Time_Cur>=TID_Timer[id].End_Time)
		{
			Delay_SetTimer(id,TID_Timer[id].Time_Delay);
			return 0x01U;
		}
	}
	return 0x00U;
}


