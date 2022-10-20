#include <rc/defs/common_defs.h>

typedef enum mbot_fram_cfg_length_t{
	WHEEL_CALIBRATION_LEN = 4 * sizeof(float), // 6 floats
	PID_VALUES_LEN = 20 * sizeof(float), // 20 floats
} mbot_fram_cfg_length_t;

typedef enum mbot_fram_cfg_offset_t{
	WHEEL_CALIBRATION_ADDR = MPU_FINAL_FRAM_ADDR, // have to start at 102 since thats where the MPU stops
	PID_VALUES_ADDR = WHEEL_CALIBRATION_ADDR + PID_VALUES_LEN,
} mbot_fram_cfg_offset_t;
