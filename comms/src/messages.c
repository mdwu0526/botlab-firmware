#include <comms/messages.h>

/*
motor_cmds_t commands serialize/deserialize
*/
int motor_cmd_t_deserialize(uint8_t* src, motor_cmds_t* dest)
{
    memcpy(dest, src, sizeof(motor_cmds_t));
    return 1;
}
int motor_cmd_t_serialize(motor_cmds_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(motor_cmds_t));
    return 1;
}


/*
motor_pid_params_t commands serialize/deserialize
*/
int motor_pid_params_t_deserialize(uint8_t* src, motor_pid_params_t* dest)
{
    memcpy(dest, src, sizeof(motor_pid_params_t));
    return 1;
}
int motor_pid_params_t_serialize(motor_pid_params_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(motor_pid_params_t));
    return 1;
}

/*
motor_pid_params_t commands serialize/deserialize
*/
int motor_states_t_deserialize(uint8_t* src, motor_states_t* dest)
{
    memcpy(dest, src, sizeof(motor_states_t));
    return 1;
}
int motor_states_t_serialize(motor_states_t* src, uint8_t* dest)
{
    memcpy(dest, src, sizeof(motor_states_t));
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