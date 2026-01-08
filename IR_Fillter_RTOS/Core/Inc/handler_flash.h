/*
 * handler_flash.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */

#ifndef INC_HANDLER_FLASH_H_
#define INC_HANDLER_FLASH_H_
#include "main.h"
#include <stdint.h>

#define FLASH_ADDR_OK  0x0801F800
#define FLASH_ADDR_NG  0x0801FC00

#define FLASH_EMPTY    65535


void Flash_Write_Array_U16(uint32_t addr, volatile uint16_t *data, uint16_t len);
uint16_t Flash_Read_U16(uint32_t addr);

#endif /* INC_HANDLER_FLASH_H_ */
