/*
 * handler_delay.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Admin
 */


#include <stdint.h>
extern volatile uint32_t timer1_counter;

volatile uint32_t delay_start = 0;
volatile uint8_t delay_active = 0;
uint32_t delay_target_ms = 0;

void Delay_ms(uint32_t ms)
{
    uint32_t start = timer1_counter;
    uint32_t target = (uint32_t)ms;

    while (1)
    {
        uint32_t elapsed;
        if (timer1_counter >= start)
            elapsed = timer1_counter - start;
        else
            elapsed = (0xFFFF - start) + timer1_counter + 1;

        if (elapsed >= target)
            break;
    }
}
