#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>

#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <hardware/adc.h>

#define INT_16_MAX 32768
#define ENCODER_RESOLUTION 20.0
#define GEAR_RATIO 20.0
#define TIMESTEP_S 1.5
#define NUM_POINTS 25

#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL -1
#define RIGHT_MOTOR_POL 1

#define LEFT_MOTOR_CHNL 1
#define RIGHT_MOTOR_CHNL 3

#define WHEEL_DIAMETER 0.08 // diameter of wheel in meters
#define PI 3.141592653589793

void blink();
void test_speed(float duty, float dtime_s);

int main()
{
    return 0;
}

void blink()
{
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}

void test_speed(float duy, float dtime_s)
{
    float left_encoder, right_encoder, left_speed, right_speed;
    rc_encoder_write(LEFT_MOTOR_CHNL, 0);
    ec_encoder_write(RIGHT_MOTOR_CHNL, 0);
    rc_motor_set(LEFT_MOTOR_CHNL, LEFT_MOTOR_POL * duty);
    rc_motor_set(RIGHT_MOTOR_CHNL, RIGHT_MOTOR_POL * duty);
    rc_nanosleep((int)(dtime_s * 1E9));
}