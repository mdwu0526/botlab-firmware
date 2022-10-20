/**
 * Copyright (c) 2021 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include <rc/encoder/encoder.h>

int main() {
    int d1, d2, d3, t1, t2, t3 = 0;

    rc_encoder_init();
    sleep_ms(2000);
    printf("delta1 | delta2 | delta3 | total1 | total2 | total3 |      \n");
    while (1) {
        d1 = rc_encoder_read_delta(1);
        d2 = rc_encoder_read_delta(2);
        d3 = rc_encoder_read_delta(3);
        t1 = rc_encoder_read_count(1);
        t2 = rc_encoder_read_count(2);
        t3 = rc_encoder_read_count(3);
        printf("\r%7d| %7d| %7d| %7d| %7d| %7d|", d1, d2, d3, t1, t2, t3);
        sleep_ms(20);
    }
}

