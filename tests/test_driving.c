#include <stdio.h>
#include <stdint.h>
#include <rc/motor/motor.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>
#include <math.h>

#define INT_16_MAX 32768
#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 3

#define LEFT_MOT_POL 1
#define RIGHT_MOT_POL 1

void example_drive_motor(float duty);
void blink();

int main()
{
    // Setup registers and pins
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    // Initialize the motors to be controlled
    rc_motor_init();

    // Blink the LED to indicate that the program is running
    blink();

    // Drive the motors
    example_drive_motor(0.5);

    blink();
    rc_motor_cleanup();
    blink();
    return 0;
}

void example_drive_motor(float duty)
{
    // Set the duty cycle of the motor
    rc_motor_set(LEFT_MOTOR_CHANNEL, (int)(duty * 0.95 * pow(2, 15)));
    rc_motor_set(RIGHT_MOTOR_CHANNEL, (int)(duty * 0.95 * pow(2, 15)));

    // Wait for 1 second
    sleep_ms((int)1e3);

    // Stop the motor
    rc_motor_set(LEFT_MOTOR_CHANNEL, 0);
    rc_motor_set(RIGHT_MOTOR_CHANNEL, 0);
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}