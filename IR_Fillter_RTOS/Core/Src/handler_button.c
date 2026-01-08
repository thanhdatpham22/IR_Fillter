/*
 * handler_button.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */

#ifndef SRC_HANDLER_BUTTON_C_
#define SRC_HANDLER_BUTTON_C_

#include "handler_button.h"
#include "handler_delay.h"

uint8_t Button_Init(Button_name_t * Button, GPIO_TypeDef  *GPIOx, uint16_t GPIO_PIN)
{
	Button->BUTTON_Port = GPIOx;
	Button->BUTTON_Pin = GPIO_PIN;
	return 1;
}
uint8_t Button_Read_Pin(Button_name_t * Button)
{
	return (HAL_GPIO_ReadPin(Button->BUTTON_Port, Button->BUTTON_Pin));
}

ButtonState_t BUTTON_Read(Button_name_t *Button)
{

	Button->State = NO_CLICK;
    while(Button_Read_Pin(Button) == NHAN)
    {
    	Button->timePress++;
    	Button->isPress = 1;
    	Delay_ms(1);
    	if (Button->timePress == SIGNLE_CLICK_TIME)
		{
			Button->State = LONGCLICK_3S;
			return Button->State;
		}
    }
    if(Button->isPress)
    {
    	while(Button_Read_Pin(Button) == NHA)
    	{
    		Button->timeDouble++;
    		Delay_ms(1);
    		if(Button->timeDouble >= DOUBLE_CLICK)
    		{
    			if(Button->timePress > DEBOUND_TIME && Button->timePress < SIGNLE_CLICK_TIME)
    			{
    				Button->State = SINGLE_CLICK;
    			}
    			else if(Button->timePress > SIGNLE_CLICK_TIME)
    			{
//					Button->State = LONGCLICK_3S;
    			}
    			Button->isPress = 0;
				Button->timePress = 0;
				Button->timeDouble = 0;
				return Button->State;
    		}
    	}
    	while(Button_Read_Pin(Button) == NHAN)
    	{
			Button->isPress = 0;
			Button->timePress = 0;
			Button->timeDouble = 0;
			Button->State = DOUBLE_CLICK;
			return Button->State;
    	}
    }
    return NO_CLICK;

}

#endif /* SRC_HANDLER_BUTTON_C_ */
