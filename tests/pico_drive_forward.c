#include <stdio.h>
#include <stdint.h>
#include <rc/motor/motor.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#define INT_16_MAX 32768

void drive_motor_up(int);
void blink();

int main()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    // adc_select_input(0);

    rc_motor_init();

    // Drive the left motor forward
    blink();
    printf("Testing left motor...\n");
    drive_motor_up(1);

    // Drive the right motor forward
    blink();
    printf("Testing right motor...\n");
    drive_motor_up(3);

    blink();
    printf("Done!\n");
    rc_motor_cleanup();

    blink();
    return 0;
}

void drive_motor_up(int motor)
{
    int32_t d = 0;
    printf("\tForward\n");
    for (; d < INT_16_MAX; d += 64)
    {
        rc_motor_set(motor, d);
        sleep_ms(4);
    }

    for (; d > 0; d -= 64)
    {
        rc_motor_set(motor, d);
        sleep_ms(4);
    }
}

void blink()
{
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
