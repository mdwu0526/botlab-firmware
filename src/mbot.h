#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/defs/common_defs.h>
#include <rc/defs/mbot_diff_defs.h>
#include <rc/fram/fram.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_messages.h>

#include <math.h>
#include <inttypes.h>

// Hardware info
#define MAX_FWD_VEL 0.8 // max forward speed (m/s)
#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"
#define MAX_TURN_VEL 2.5 // max turning speed (rad/s)

// TODO: Enter the polarity values for your motors and encoders
#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL 1
#define RIGHT_MOTOR_POL 1

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 0

// data to hold current mpu state (not used)
static rc_mpu_data_t mpu_data;
static i2c_inst_t *i2c;

uint64_t timestep_us = 0;

// data to hold calibration coefficients
float coeffs[4];

// data to hold the PID values
static mbot_pid_gains_t mbot_pid_gains;

typedef struct pid_parameters pid_parameters_t;
struct pid_parameters
{
    float kp;
    float ki;
    float kd;
    float dFilterHz;
};

float clamp_duty(float duty);

float clamp_angle(float angle);

float check_sign(float num);

float open_loop_control(int MOTOR_CHANNEL, float SET_SPEED);

float pid_control(int MOTOR_CHANNEL, float SET_SPEED, float MEASURED_SPEED, rc_filter_t *integrator, rc_filter_t *input_f, rc_filter_t *pid, pid_parameters_t params);

float gyrodometry(float MPU_HEADING, float ODOM_HEADING, float THRESHOLD, rc_filter_t lp, rc_filter_t hp);

float odometry_imu_fusion (mbot_imu_t imu, odometry_t odometry, rc_filter_t lpf, rc_kalman_t kf);

// data to hold the IMU results
mbot_imu_t current_imu = {0};
// data to hold the received timestamp
timestamp_t received_time = {0};
// current odometry state
odometry_t current_odom = {0};
// current encoder states
mbot_encoder_t current_encoders = {0};
// current body frame command
mbot_motor_command_t current_cmd = {0};

/**
 * Example filter and PID parameter initialization
 *
 * rc_filter_t my_filter;
 *
 * pid_parameters_t pid_params = {
 *    .kp = 1.0,
 *    .ki = 0.0,
 *    .kd = 0.0,
 *    .dFilterHz = 25.0
 * };
 */

rc_filter_t left_pid;
rc_filter_t right_pid;
rc_filter_t fwd_vel_pid;
rc_filter_t turn_vel_pid;

// Additional Filters for PID Control
rc_filter_t setpoint;
rc_filter_t left_pid_integrator;
rc_filter_t right_pid_integrator;

// Filter for Sensor Fusion
rc_filter_t sensor_fusion_lp;
rc_filter_t sensor_fusion_hp;
rc_filter_t gyro_integrator;


pid_parameters_t left_pid_params = {
    .kp = 4.0,
    .ki = 0.5,
    .kd = 0,
    .dFilterHz = MAIN_LOOP_HZ/2,
};
pid_parameters_t right_pid_params = {
    .kp = 4.0,
    .ki = 0.5,
    .kd = 0,
    .dFilterHz = MAIN_LOOP_HZ/2,
};
pid_parameters_t fwd_vel_pid_params = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
    .dFilterHz = 10.0,
};
pid_parameters_t turn_vel_pid_params = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
    .dFilterHz = 10.0,
};

// float clamp_duty(float duty);

#endif
