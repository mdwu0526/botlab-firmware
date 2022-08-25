#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <comms/messages.h>
#include <rc/math/filter.h>

#ifndef _H_CONTROLLER
#define _H_CONTROLLER

#define CONTROLLER_TIMESTEP 0.5
#define EPS 1e-5

typedef double (*MotorFeedforward)(motor_cmds_t setpoints, motor_states_t states);


int init_controllers(rc_filter_t** base_filters, motor_pid_params_t pid_params);
double run_controller(rc_filter_t** filters, motor_pid_params_t pid_params, motor_cmds_t setpoints, motor_states_t states, 
    MotorFeedforward position_ff, MotorFeedforward velocity_ff, MotorFeedforward current_ff);
#endif