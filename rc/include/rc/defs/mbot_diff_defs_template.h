#include <rc/defs/common_defs.h>

#ifndef RC_DIFF_DEFS_H
#define RC_DIFF_DEFS_H

#define LEFT_MOTOR_CHANNEL      1
#define RIGHT_MOTOR_CHANNEL     3
#define WHEEL_RADIUS            0.04      // change your wheel radius to your specific measurements
#define WHEEL_BASE              0.16      // change your wheelbase to your specific measurements

#define M1_SLOPE				0.00580285 		  // define your mbot-specific motor 1 slope (duty cycle / wheel speed)
#define M1_INT					0.11208425 		  // define your mbot-specific motor 1 intercept (duty cycle to get motor moving)
#define M3_SLOPE 				0.00576415 		  // define your mbot-specific motor 3 slope (duty cycle / wheel speed)
#define M3_INT 					0.1011394 		  // define your mbot-specific motor 3 intercept (duty cycle to get motor moving)

typedef enum mbot_fram_cfg_length_t{
	WHEEL_CALIBRATION_LEN = 8 * sizeof(float), // 8 floats
	PID_VALUES_LEN = 20 * sizeof(float), // 20 floats
} mbot_fram_cfg_length_t;

typedef enum mbot_fram_cfg_offset_t{
	WHEEL_CALIBRATION_ADDR = MPU_FINAL_FRAM_ADDR, // have to start at 102 since thats where the MPU stops
	PID_VALUES_ADDR = WHEEL_CALIBRATION_ADDR + WHEEL_CALIBRATION_LEN,
} mbot_fram_cfg_offset_t;

#endif