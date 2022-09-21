#include <stdio.h>
#include <stdint.h>
// #include <memory.h>
#include <string.h>
#include <pico/binary_info.h> 

#ifndef MESSAGES_H
#define MESSAGES_H

typedef struct motor_cmds{
    int32_t pwm_abs; // raw pwm cmd/offset
    int32_t i_sp;   // current setpoint
    int32_t v_sp;   // velocity setpoint
    int32_t x_sp;   // position setpoint
} motor_cmds_t;

int motor_cmd_t_deserialize(uint8_t* src, motor_cmds_t* dest);
int motor_cmd_t_serialize(motor_cmds_t* src, uint8_t* dest);

typedef struct motor_pid_params{
    int32_t kp_i;   // current pid params
    int32_t ki_i;
    int32_t kd_i;
    int32_t kp_v;   // velocity pid params
    int32_t ki_v;
    int32_t kd_v;
    int32_t kp_x;   // position pid params
    int32_t ki_x;
    int32_t kd_x;
    double motor_kv;
    double motor_kt;
    double J;       // mmoi on motor shaft output
    double b;       // rotational friction dampening coefficient
} motor_pid_params_t;

int motor_pid_params_t_deserialize(uint8_t* src, motor_pid_params_t* dest);
int motor_pid_params_t_serialize(motor_pid_params_t* src, uint8_t* dest);

typedef struct motor_states{
    int32_t i; // current (scaled)
    int32_t v; // velocity (ticks / sec)
    int32_t x; // position (ticks)
} motor_states_t;

int motor_states_t_deserialize(uint8_t* src, motor_states_t* dest);
int motor_states_t_serialize(motor_states_t* src, uint8_t* dest);

typedef struct mbot_imu
{
    int64_t utime;

    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float tb_x;
    float tb_y;
    float tb_z;
    float temp;
    
} mbot_imu_t;

int mbot_imu_t_deserialize(uint8_t* src, mbot_imu_t* dest);
int mbot_imu_t_serialize(mbot_imu_t* src, uint8_t* dest);

#endif
