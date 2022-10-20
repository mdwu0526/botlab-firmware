#include <comms/mbot_messages.h>

/*
timestamp_t commands serialize/deserialize
*/
int timestamp_t_deserialize(uint8_t* src, timestamp_t* dest)
{
    memcpy(dest, src, sizeof(timestamp_t));
    return 1;
}
int timestamp_t_serialize(timestamp_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(timestamp_t));
    return 1;
}

/*
odometry_t commands serialize/deserialize
*/
int odometry_t_deserialize(uint8_t* src, odometry_t* dest)
{
    memcpy(dest, src, sizeof(odometry_t));
    return 1;
}
int odometry_t_serialize(odometry_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(odometry_t));
    return 1;
}

/*
mbot_imu_t commands serialize/deserialize
*/
int mbot_imu_t_deserialize(uint8_t* src, mbot_imu_t* dest)
{
    memcpy(dest, src, sizeof(mbot_imu_t));
    return 1;
}
int mbot_imu_t_serialize(mbot_imu_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mbot_imu_t));
    return 1;
}

/*
mbot_encoder_t commands serialize/deserialize
*/
int mbot_encoder_t_deserialize(uint8_t* src, mbot_encoder_t* dest)
{
    memcpy(dest, src, sizeof(mbot_encoder_t));
    return 1;
}
int mbot_encoder_t_serialize(mbot_encoder_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mbot_encoder_t));
    return 1;
}

/*
omni_encoder_t commands serialize/deserialize
*/
int omni_encoder_t_deserialize(uint8_t* src, omni_encoder_t* dest)
{
    memcpy(dest, src, sizeof(omni_encoder_t));
    return 1;
}
int omni_encoder_t_serialize(omni_encoder_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(omni_encoder_t));
    return 1;
}

/*
mbot_motor_command_t commands serialize/deserialize
*/
int mbot_motor_command_t_deserialize(uint8_t* src, mbot_motor_command_t* dest)
{
    memcpy(dest, src, sizeof(mbot_motor_command_t));
    return 1;
}
int mbot_motor_command_t_serialize(mbot_motor_command_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mbot_motor_command_t));
    return 1;
}

/*
omni_motor_command_t commands serialize/deserialize
*/
int omni_motor_command_t_deserialize(uint8_t* src, omni_motor_command_t* dest)
{
    memcpy(dest, src, sizeof(omni_motor_command_t));
    return 1;
}
int omni_motor_command_t_serialize(omni_motor_command_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(omni_motor_command_t));
    return 1;
}

/*
mbot_pid_gains_t commands serialize/deserialize
*/
int mbot_pid_gains_t_deserialize(uint8_t* src, mbot_pid_gains_t* dest)
{
    memcpy(dest, src, sizeof(mbot_pid_gains_t));
    return 1;
}
int mbot_pid_gains_t_serialize(mbot_pid_gains_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(mbot_pid_gains_t));
    return 1;
}