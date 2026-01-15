/*
 * handler_flash.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */
#include "handler_flash.h"
#include "cmsis_os.h"
void Flash_Write_Array_U16(uint32_t addr, volatile uint16_t *data, uint16_t len)
{
    HAL_FLASH_Unlock();

    /* Xóa page chứa addr */
    FLASH_EraseInitTypeDef erase;
    uint32_t pageError;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = addr;
    erase.NbPages     = 1;
    taskENTER_CRITICAL();
    HAL_FLASHEx_Erase(&erase, &pageError);

    /* Ghi từng phần tử */
    for(uint16_t i = 0; i < len; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          addr + i * 2,
                          data[i]);
    }
    taskEXIT_CRITICAL();
    HAL_FLASH_Lock();
}


uint16_t Flash_Read_U16(uint32_t addr)
{
    return *(volatile uint16_t*)addr;
}
