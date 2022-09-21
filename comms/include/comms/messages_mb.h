#include <stdio.h>
#include <stdint.h>
// #include <memory.h>
#include <string.h>
#include <pico/binary_info.h> 

#ifndef MESSAGES_MB_H
#define MESSAGES_MB_H

enum message_topics{
    MBOT_TIMESYNC = 201, 
    MBOT_PIDS = 202, 
    ODOMETRY = 210, 
    RESET_ODOMETRY = 211, 
    MBOT_IMU = 220, 
    MBOT_MOTOR_COMMAND = 230, 
    OMNI_MOTOR_COMMAND = 230, 
    MBOT_ENCODERS = 240, 
    OMNI_ENCODERS = 241, 
    RESET_ENCODERS = 242
};

typedef struct timestamp{
    uint64_t utime; // timestamp
} timestamp_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) odometry{
    uint64_t utime; // timestamp
    float x;
    float y;
    float theta;
} odometry_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_imu{
    uint64_t utime; // timestamp
    float gyro[3];
    float accel[3];
    float mag[3];
    float tb[3];
    float temperature;
} mbot_imu_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_encoder{
    uint64_t utime; // timestamp
    int64_t leftticks;
    int64_t rightticks;
    int16_t left_delta;
    int16_t right_delta;
} mbot_encoder_t;

typedef struct __attribute__((__packed__ )) omni_encoder{
    uint64_t utime; // timestamp
    int64_t aticks;
    int64_t bticks;
    int64_t cticks;
    int16_t a_delta;
    int16_t b_delta;
    int16_t c_delta;
} omni_encoder_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_motor_command{
    uint64_t utime; // timestamp
    float trans_v;
    float angular_v;
} mbot_motor_command_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) omni_motor_command{
    uint64_t utime; // timestamp
    float vx;
    float vy;
    float wz;
} omni_motor_command_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_pid_gains{
    float motor_a_kp;
    float motor_a_ki;
    float motor_a_kd;
    float motor_a_Tf;

    float motor_b_kp;
    float motor_b_ki;
    float motor_b_kd;
    float motor_b_Tf;

    float motor_c_kp;
    float motor_c_ki;
    float motor_c_kd;
    float motor_c_Tf;

    float bf_trans_kp;
    float bf_trans_ki;
    float bf_trans_kd;
    float bf_trans_Tf;

    float bf_rot_kp;
    float bf_rot_ki;
    float bf_rot_kd;
    float bf_rot_Tf;
} mbot_pid_gains_t;

int timestamp_t_deserialize(uint8_t* src, timestamp_t* dest);
int timestamp_t_serialize(timestamp_t* src, uint8_t* dest);

int odometry_t_deserialize(uint8_t* src, odometry_t* dest);
int odometry_t_serialize(odometry_t* src, uint8_t* dest);

int mbot_imu_t_deserialize(uint8_t* src, mbot_imu_t* dest);
int mbot_imu_t_serialize(mbot_imu_t* src, uint8_t* dest);

int mbot_encoder_t_deserialize(uint8_t* src, mbot_encoder_t* dest);
int mbot_encoder_t_serialize(mbot_encoder_t* src, uint8_t* dest);

int omni_encoder_t_deserialize(uint8_t* src, omni_encoder_t* dest);
int omni_encoder_t_serialize(omni_encoder_t* src, uint8_t* dest);

int mbot_motor_command_t_deserialize(uint8_t* src, mbot_motor_command_t* dest);
int mbot_motor_command_t_serialize(mbot_motor_command_t* src, uint8_t* dest);

int omni_motor_command_t_deserialize(uint8_t* src, omni_motor_command_t* dest);
int omni_motor_command_t_serialize(omni_motor_command_t* src, uint8_t* dest);

int mbot_pid_gains_t_deserialize(uint8_t* src, mbot_pid_gains_t* dest);
int mbot_pid_gains_t_serialize(mbot_pid_gains_t* src, uint8_t* dest);

#endif
