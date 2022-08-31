#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>

#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>

#define INT_16_MAX 32768

void blink();

int main()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    rc_motor_init();
    rc_encoder_init();

    while (true)
    {
        blink();
        int new_value0, total0;
        int new_value2, total2;

        // Read the delta encoder ticks
        new_value0 = rc_encoder_read_delta(1);
        new_value2 = rc_encoder_read_delta(3);

        // Read the total encoder ticks
        total0 = rc_encoder_read_count(1);
        total2 = rc_encoder_read_count(3);

        // Display on /dev/ttyACM0
        printf("\033[2A\r| Delta L | Delta R | Total L | Total R |\n\r| %7d | %7d | %7d | %7d |", new_value0, new_value2, total0, total2);
    }

    return 0;
}

void blink()
{
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}