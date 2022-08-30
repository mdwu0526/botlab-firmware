#include <stdio.h>
#include <stdint.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <pico/stdlib.h>
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

#define WHEEL_DIAMETER 0.08 // diameter of wheel in meters
#define PI 3.141592653589793

void blink();

int main() {
    const float I_conversion_factor = 2 * 3.3f / (1 << 12);
    const float RPM_conversion_factor = WHEEL_DIAMETER * PI / (GEAR_RATIO * TIMESTEP_S * ENCODER_RESOLUTION);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    rc_motor_init();
    rc_encoder_init();
    blink();
    sleep_ms(1000*TIMESTEP_S);
    int32_t d = 0;
    int encoder_reading_l, encoder_reading_r;
    float current_reading;
    float wheel_speed_l, wheel_speed_r;
    printf("\nDuty (PWM),Left Speed (m/s),Right Speed (m/s)\n");
    adc_select_input(0);
    for (; d < INT_16_MAX; d += INT_16_MAX/NUM_POINTS) {
        rc_motor_set(1, LEFT_MOTOR_POL*d);
        rc_motor_set(3, -1*RIGHT_MOTOR_POL*d);
        encoder_reading_l = LEFT_ENC_POL * rc_encoder_read_delta(1);
        encoder_reading_r = RIGHT_ENC_POL * rc_encoder_read_delta(3);
        wheel_speed_l = RPM_conversion_factor * encoder_reading_l;
        wheel_speed_r = RPM_conversion_factor * encoder_reading_r;
        // current_reading = 0.0;
        // for(int i=0; i<10; i++){
        //     current_reading += I_conversion_factor * adc_read()/10;
        // }
        printf("%f,%f,%f\n", (float)d/(float)INT_16_MAX, wheel_speed_l, wheel_speed_r);
        sleep_ms(1000*TIMESTEP_S);
    }
    rc_motor_set(1, 0);
    rc_motor_set(3, 0);
    
    blink();
    printf("\nDone!\n");
    
    rc_motor_cleanup(); 
    blink();
    return 0;
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
