/*
 * handler_button.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */

#ifndef INC_HANDLER_BUTTON_H_
#define INC_HANDLER_BUTTON_H_
#include "main.h"

#define DEBOUND_TIME 50
#define SIGNLE_CLICK_TIME 3000
#define DOUBLE_CLICK_TIME 500
#define NHAN 0
#define NHA 1


#define BTN_OK  1
#define BTN_NG  2

typedef enum {
    NO_CLICK = 0,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONGCLICK_3S
} ButtonState_t;

typedef struct{
	GPIO_TypeDef *BUTTON_Port;
	uint16_t BUTTON_Pin;
	ButtonState_t  State;
	uint8_t isPress;
	uint16_t timePress;
	uint16_t timeDouble;

}Button_name_t;


uint8_t Button_Init(Button_name_t * Button, GPIO_TypeDef  *GPIOx, uint16_t GPIO_PIN);
uint8_t Button_Read_Pin(Button_name_t * Button);
ButtonState_t BUTTON_Read(Button_name_t *Button);

#endif /* INC_HANDLER_BUTTON_H_ */
