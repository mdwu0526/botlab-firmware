/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include <pico/stdio_usb.h>

int main() 
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    stdio_init_all();
    char char1 = 0x00;
    char char2 = 0x00;
    sleep_ms(2000);
    int resp_char = PICO_ERROR_NO_DATA;
    while (1)
    {
        // get whats typed one one serial, and broadcast to the other
        resp_char = stdio_usb_in_chars_itf(0, &char1, 1);
        if(resp_char != PICO_ERROR_NO_DATA)
        {
            stdio_usb_out_chars_itf(1, &char1, 1);            
        }
        resp_char = stdio_usb_in_chars_itf(1, &char1, 1);
        if(resp_char != PICO_ERROR_NO_DATA)
        {
            stdio_usb_out_chars_itf(0, &char1, 1);            
        }
    }
}

