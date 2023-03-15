/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_messages.h>
#include <rc/math/kalman.h>

#include <math.h>
#include <inttypes.h>
#include "mbot.h"


// MOTOR 1 = LEFT MOTOR
// MOTOR 3 = RIGHT MOTOR

#define LED_PIN 25
#define MAIN_LOOP_HZ 25.0 // 25 hz loop
#define MAIN_LOOP_PERIOD (1.0f / MAIN_LOOP_HZ)

// data to hold current mpu state
static rc_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

const float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));
const float rad2RPM = 60 / (2.0 * PI); // Converts rad/s to RPM
const float RPM2rad = 1/rad2RPM; // Converts from RPM to rad/s
const float deg2rad = PI/180;

// Global constants for reading output
float left_speed;
float right_speed;
float left_error;
float right_error;
float left_duty;
float right_duty;
float measured_f_spd;
float measured_t_spd;
float heading;
float theta_odom = 0;
float theta_gyro = 0;

void timestamp_cb(timestamp_t *received_timestamp)
{
    // if we havent set the offset yet
    if (timestamp_offset == 0)
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t *received_encoder_vals)
{
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}

void reset_odometry_cb(odometry_t *received_odom)
{
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

int write_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];
    memcpy(pid_bytes, &mbot_pid_gains, PID_VALUES_LEN);
    return rc_write_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, &pid_bytes[0]);
}

void pid_values_cb(mbot_pid_gains_t *received_pid_gains)
{
    memcpy(&mbot_pid_gains, received_pid_gains, sizeof(mbot_pid_gains_t));
    write_pid_coefficients(i2c);
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)&reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, (MsgCb)&reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize)&mbot_motor_command_t_deserialize, (Serialize)&mbot_motor_command_t_serialize, NULL);
    // PID values topic
    comms_register_topic(MBOT_PIDS, sizeof(mbot_pid_gains_t), (Deserialize)&mbot_pid_gains_t_deserialize, (Serialize)&mbot_pid_gains_t_serialize, (MsgCb)&pid_values_cb);
}

void read_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];

    if (rc_read_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, pid_bytes) > 0)
    {
        printf("reading fram success.\n");
        memcpy(&mbot_pid_gains, pid_bytes, PID_VALUES_LEN);
        printf("read gains from fram!\r\n");
    }
    else
    {
        printf("reading PID gains from fram failure.\n");
    }
}

bool timer_cb(repeating_timer_t *rt)
{
    // Read the PID values
    if (comms_get_topic_data(MBOT_PIDS, &mbot_pid_gains))
    {
        uint64_t msg_time = current_pico_time;
        // Print the PID values
        printf("Left: %f, %f, %f, %f", mbot_pid_gains.motor_a_kp,
               mbot_pid_gains.motor_a_ki,
               mbot_pid_gains.motor_a_kd,
               1.0 / mbot_pid_gains.motor_a_Tf);

        // update the PID gains of the left motor PID controller
        rc_filter_pid(&left_pid,
                      mbot_pid_gains.motor_a_kp,
                      mbot_pid_gains.motor_a_ki,
                      mbot_pid_gains.motor_a_kd,
                      1.0 / mbot_pid_gains.motor_a_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the right motor PID controller
        rc_filter_pid(&right_pid,
                      mbot_pid_gains.motor_c_kp,
                      mbot_pid_gains.motor_c_ki,
                      mbot_pid_gains.motor_c_kd,
                      1.0 / mbot_pid_gains.motor_c_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame controller
        rc_filter_pid(&fwd_vel_pid,
                      mbot_pid_gains.bf_trans_kp,
                      mbot_pid_gains.bf_trans_ki,
                      mbot_pid_gains.bf_trans_kd,
                      1.0 / mbot_pid_gains.bf_trans_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame rotation controller
        rc_filter_pid(&turn_vel_pid,
                      mbot_pid_gains.bf_rot_kp,
                      mbot_pid_gains.bf_rot_ki,
                      mbot_pid_gains.bf_rot_kd,
                      1.0 / mbot_pid_gains.bf_rot_Tf,
                      1.0 / MAIN_LOOP_HZ);
    }
    // only run if we've received a timesync message...
    if (comms_get_topic_data(MBOT_TIMESYNC, &received_time))
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        uint64_t latency_time = cur_pico_time - current_pico_time;
        current_pico_time = cur_pico_time;
        // first, get the IMU data and send across the wire
        current_imu.utime = cur_pico_time; // received_time.utime;

        // read the encoders
        int enc_cnt_l = LEFT_ENC_POL * rc_encoder_read_count(LEFT_MOTOR_CHANNEL);
        int enc_delta_l = LEFT_ENC_POL * rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
        int enc_cnt_r = RIGHT_ENC_POL * rc_encoder_read_count(RIGHT_MOTOR_CHANNEL);
        int enc_delta_r = RIGHT_ENC_POL * rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);
        current_encoders.utime = cur_pico_time; // received_time.utime;
        current_encoders.right_delta = enc_delta_r;
        current_encoders.rightticks = enc_cnt_r;
        current_encoders.left_delta = enc_delta_l;
        current_encoders.leftticks = enc_cnt_l;

        /*************************************************************
         * TODO:
         *      - Populate the odometry messages.
         *          -The struct for odometry_t is defined in comms/include/comms/messages_mb.h (DO NOT EDIT THIS FILE)
         *      - Note that the way we compute the displacement of the motor is from encoder readings and
         *        that we convert encoder readings to meters using the enc2meters variable defined above.
         *      - Use the equations provided in the document to compute the odometry components
         *      - Remember to clamp the orientation between [0, 2pi]!
         *************************************************************/
        float delta_d, delta_theta, delta_s_l, delta_s_r; // displacement in meters and rotation in radians
        delta_s_r = enc_delta_r*enc2meters;
        delta_s_l = enc_delta_l*enc2meters;
        delta_theta = (delta_s_r - delta_s_l)/WHEEL_BASE;
        delta_d = (delta_s_r + delta_s_l)/2;
        float delta_x = delta_d * cos(current_odom.theta + delta_theta/2);
        float delta_y = delta_d * sin(current_odom.theta + delta_theta/2);
        theta_odom = clamp_angle(current_odom.theta + delta_theta); // Theta based on odometry
        theta_gyro = clamp_angle(rc_filter_march(&gyro_integrator,mpu_data.gyro[2])*deg2rad); // Theta based on fused MPU
        
        current_odom.x += delta_x; // delta_s_r; 
        current_odom.y += delta_y; // delta_s_l;
        current_odom.utime = cur_pico_time;
        heading = theta_odom;
        current_odom.theta = gyrodometry(theta_gyro,theta_odom,0.15,sensor_fusion_lp,sensor_fusion_hp);
        /*************************************************************
         * End of TODO
         *************************************************************/

        // get the current motor command state (if we have one)
        if (comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd))
        {
            int16_t l_cmd, r_cmd;                 // left and right motor commands
            float left_sp, right_sp;              // speed in m/s
            float measured_vel_l, measured_vel_r; // measured velocity in m/s
            float l_duty, r_duty;                 // duty cycle in range [-1, 1]
            float dt = MAIN_LOOP_PERIOD;          // time since last update in seconds
            if (OPEN_LOOP)
            {
                /*************************************************************
                 * TODO:
                 *      - Implement the open loop motor controller to compute the left
                 *          and right wheel commands
                 *      - Determine the setpoint velocities for left and right motor using the wheel velocity model
                 *      - To compute the measured velocities, use dt as the timestep (∆t)
                 ************************************************************/
                
                // Convert setpoint velocities into m/s
                left_sp = (current_cmd.trans_v - current_cmd.angular_v*(WHEEL_BASE/2));
                right_sp = (current_cmd.trans_v + current_cmd.angular_v*(WHEEL_BASE/2));

                // Calibration curve takes in m/s and outputs Duty Cycle [0,1]
                l_duty = open_loop_control(LEFT_MOTOR_CHANNEL,left_sp);
                r_duty = open_loop_control(RIGHT_MOTOR_CHANNEL,right_sp);

                measured_vel_l = delta_s_l/dt; // Displacement/time [m/s]
                measured_vel_r = delta_s_r/dt; // Displacement/time [m/s]
                /*************************************************************
                 * End of TODO
                 *************************************************************/
            }
            else
            {
                /*************************************************************
                 * TODO:
                 *      - Implement the closed loop motor controller to compute the left
                 *          and right wheel commands
                 *      - To calculate the measured velocity, use MAIN_LOOP_PERIOD or latency_time
                 *          as the timestep
                 *      - Compute the error between the setpoint velocity and the measured velocity
                 *      - We recommend to use the open loop controller as part of the closed loop to improve
                 *          performance.
                 *          Example: open_loop_control(LEFT_MOTOR_CHANNEL, left_sp)
                 *      - Use the PID filters defined in mbot.h and main() function to calculate desired
                 *          duty, use rc_filter_march() function.
                 *          Example: rc_filter_march(&left_pid, left_error)
                 * TODO:
                 *      - Compute the error between the target and measured translation+rotation velocity
                 *      - Compute the new forward and rotation setpoints that will be used in
                 *          the wheel speed PID (these should be written in lines above
                 *          the previous controller)
                 *      - Update and the fwd_sp and turn_sp variables for this.
                 *
                 ************************************************************/
                float fwd_sp, turn_sp;                     // forward and turn setpoints in m/s and rad/s
                float measured_vel_fwd, measured_vel_turn; // measured forward and turn velocities in m/s and rad/s
                
                // Convert setpoint velocities into m/s
                left_sp = (current_cmd.trans_v - current_cmd.angular_v*(WHEEL_BASE/2));
                right_sp = (current_cmd.trans_v + current_cmd.angular_v*(WHEEL_BASE/2));

                // Convert measured velocity to m/s
                measured_vel_l = (delta_s_l/dt);
                measured_vel_r = (delta_s_r/dt);

                l_duty = pid_control(LEFT_MOTOR_CHANNEL,left_sp,measured_vel_l,&left_pid_integrator,&setpoint,&left_pid,left_pid_params);
                r_duty = pid_control(RIGHT_MOTOR_CHANNEL,right_sp,measured_vel_r,&right_pid_integrator,&setpoint,&right_pid,right_pid_params);

                left_speed = measured_vel_l;
                right_speed = measured_vel_r;
                left_error = left_sp-measured_vel_l;
                right_error = right_sp-measured_vel_r;
                measured_f_spd = (measured_vel_l+measured_vel_r)/2;
                measured_t_spd = (measured_vel_r-measured_vel_l)/WHEEL_BASE;
                left_duty = l_duty;
                right_duty = r_duty;

                /**
                 *  Example closed loop controller
                 *      (assuming the error between the target and measured is computed)
                 *
                 * float pid_delta_vel = rc_filter_march(&pid_filter, error);
                 * float desired_vel = commanded_val + pid_delta_vel;
                 */

                /*************************************************************
                 * End of TODO
                 *************************************************************/
            }
            // Clamp duty cycle to [-1, 1]
            l_duty = clamp_duty(l_duty);
            r_duty = clamp_duty(r_duty);

            // duty to motor command
            l_cmd = LEFT_MOTOR_POL * (int)(l_duty * 0.95 * pow(2, 15));
            r_cmd = RIGHT_MOTOR_POL * (int)(r_duty * 0.95 * pow(2, 15));

            // set left and right motor command
            rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
            rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &current_encoders);
        // send odom on wire
        comms_write_topic(ODOMETRY, &current_odom);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &current_imu);
        uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }

    return true;
}

int main()
{
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));
    
    set_sys_clock_khz(250000, true); // set master clock to 250MHz (if problematic try 125Mhz)
    stdio_init_all(); // enable USB serial terminal
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    printf("\nMBot Booting Up!\n");

    printf("initializinging motors...\n");
    rc_motor_init();
    printf("initializinging encoders...\n");
    rc_encoder_init();

    // Pins
    // for the i2c to the IMU
    const uint sda_pin = 4;
    const uint scl_pin = 5;

    // Ports
    i2c = i2c0;
    // Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting i2c functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    printf("setting i2c pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the IMU using the Digital Motion Processor
    printf("initializing DMP...\n");
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro = 1;
    mpu_config.enable_magnetometer = 1;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    
    // Calibrate the gyro to eliminate bias, Mbot must be still for this
    rc_mpu_calibrate_gyro_routine(mpu_config);
    sleep_ms(500);
    rc_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(rc_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &rc_dmp_callback);
    printf("MPU Initialized!\n");

    // create topics and register the serialize/deserialize functions
    printf("init comms...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    int running = 1;

    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms

    printf("Done Booting Up!\n\n");

    /*************************************************************
     * TODO:
     *  - wheel speed PID
     *      - Initilize the PID Filters rc_filter_empty()
     *      - Set the PID gains rc_filter_pid()
     * TODO:
     *  - body frame velocity PID
     *      - Initialize the PID filters for translation and rotation vel
     *      - Set the PID gains for translation and rotation vel
     *
     *************************************************************/

    // Example initialization of a PID filter defined in mbot.h
    // my_filter = rc_filter_empty();

    // Example of assigning PID parameters (using pid_parameters_t from mbot.h)
    // rc_filter_pid(&my_filter,
    //             pid_params.kp,
    //             pid_params.ki,
    //             pid_params.kd,
    //             1.0 / pid_params.dFilterHz,
    //             1.0 / MAIN_LOOP_HZ);

    // Example of setting limits to the output of the filter
    // rc_filter_enable_saturation(&my_filter, min_val, max_val);

    //
    setpoint = rc_filter_empty();
    float tc = 0.02; // Time constant in seconds
    rc_filter_first_order_lowpass(&setpoint,MAIN_LOOP_PERIOD,tc);

    // Configure left and right PID filters
    left_pid = rc_filter_empty();
    right_pid = rc_filter_empty();
    left_pid_integrator = rc_filter_empty();
    right_pid_integrator = rc_filter_empty();

    // Configures the integral part of the PID controller separately to incorporate saturation
    rc_filter_integrator(&left_pid_integrator,MAIN_LOOP_PERIOD);
    rc_filter_integrator(&right_pid_integrator,MAIN_LOOP_PERIOD);
    
    // Configurator the saturator for the integrators
    float SAT_MIN = -1;
    float SAT_MAX = 1;
    rc_filter_enable_saturation(&left_pid_integrator,SAT_MIN,SAT_MAX);
    rc_filter_enable_saturation(&right_pid_integrator,SAT_MIN,SAT_MAX);

    // Integral gains are set to 0 because there is a separate filter to handle the integral gain
    rc_filter_pid(&left_pid,left_pid_params.kp,0,left_pid_params.kd,1.0/left_pid_params.dFilterHz,MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_pid,right_pid_params.kp,0,right_pid_params.kd,1.0/right_pid_params.dFilterHz,MAIN_LOOP_PERIOD);

    gyro_integrator = rc_filter_empty();
    sensor_fusion_lp = rc_filter_empty();
    sensor_fusion_hp = rc_filter_empty();

    // TODO: Tune these values for better odometry heading
    float freq = 2*PI*(2); // 2 Hz crossover frequency 
    float damp = 1; // Critcally damped 
    rc_filter_third_order_complement(&sensor_fusion_lp,&sensor_fusion_hp,freq,damp,MAIN_LOOP_PERIOD);
    rc_filter_integrator(&gyro_integrator,MAIN_LOOP_PERIOD);
    // // initialize the low pass filter and pass the imu data in
    // rc_filter_t lpf = rc_filter_empty();
    // float tc = 0.00001; // Time constant, time it takes to reach 63.4% of the original value
    // rc_filter_first_order_lowpass(&lpf,MAIN_LOOP_PERIOD,tc);

    // // initialize the kalman filter
    // rc_kalman_t kf = rc_kalman_empty();
    // rc_matrix_t F;
    // rc_matrix_t G;
    // rc_matrix_t H;
    // rc_matrix_t Q;
    // rc_matrix_t R;
    // rc_matrix_t Pi;
    // rc_kalman_alloc_lin(&kf, F, G, H, Q, R, Pi);

    /*************************************************************
     * End of TODO
     *************************************************************/

    if (OPEN_LOOP)
    {
        printf("Running in open loop mode\n");
    }
    else
    {
        printf("Running in closed loop mode\n");
    }

    while (running)
    {
        printf("\033[2A\r|      SENSORS      |           ODOMETRY          |     SETPOINTS     |\n\r|  L_ENC  |  R_ENC  |    X    |    Y    |    θ    |   FWD   |   ANG   |   LSPD   |   RSPD   |   L_ER   |   R_ER   |   F_SP   |   T_SP   |   HEADG   |\n\r|%7lld  |%7lld  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f   |%7.3f   |%7.3f   |%7.3f   |%7.3f   |%7.3f   |%7.3f   |", current_encoders.leftticks, current_encoders.rightticks, current_odom.x, current_odom.y, current_odom.theta, current_cmd.trans_v, current_cmd.angular_v, left_speed, right_speed, left_error, right_error, measured_f_spd, measured_t_spd, heading);
    }
}

/**
 * @brief Clamp duty cycle between -1 and 1. If not applied, robot drives in reverse only
 *
 * @param duty
 */
float clamp_duty(float duty)
{
    if (duty > 1.0)
    {
        return 1.0;
    }
    else if (duty < -1.0)
    {
        return -1.0;
    }
    return duty;
}

/**
 * @brief Clamp angle between 0 and 2pi.
 *
 * @param angle
 */
float clamp_angle(float angle)
{
    if (angle > 2*PI)
    {
        return fmod(angle,2*PI);
    }
    else if (angle < 0)
    {
        return 2*PI+(fmod(angle,2*PI)); // the fmod remainder would be negative, since it's less than 0, so it should be + (-)
    }
    return angle;
}



/**
 * @brief Takes the sign of a number
 *
 * @param num
 */
float check_sign(float num)
{
    if (num > 0.0)
    {
        return 1.0;
    }
    else if (num == 0.0)
    {
        return 0.0;
    }
    else 
    {
        return -1.0;
    }
}

/**
 * @brief Open loop controller for MBot
 * Takes in speed variable in m/s and returns duty cycle [0,1]
 *
 * @param MOTOR_CHANNEL
 * @param SET_SPEED
 * 
 */
float open_loop_control(int MOTOR_CHANNEL, float SET_SPEED){
    float val = 0;
    SET_SPEED = SET_SPEED/WHEEL_RADIUS*rad2RPM;
    if(MOTOR_CHANNEL == LEFT_MOTOR_CHANNEL){
        val = check_sign(M1_SLOPE*SET_SPEED)*(fabs(M1_SLOPE*SET_SPEED) + fabs(M1_INT));
    }
    else if(MOTOR_CHANNEL == RIGHT_MOTOR_CHANNEL){
        val = check_sign(M3_SLOPE*SET_SPEED)*(fabs(M3_SLOPE*SET_SPEED) + fabs(M3_INT));
    }
    return val;
}

/**
 * @brief PID controller for MBot to control individual wheel velocities
 * Takes in setpoint speed and measured speed in m/s and returns duty cycle [0,1]
 *
 * @param MOTOR_CHANNEL
 * @param SET_SPEED
 * @param MEASURED_SPEED
 * @param input_f
 * @param integrator
 * @param pid
 * 
 */
float pid_control(int MOTOR_CHANNEL, float SET_SPEED, float MEASURED_SPEED, rc_filter_t *integrator, rc_filter_t *input_f, rc_filter_t *pid, pid_parameters_t params){

    // Pass setpoint through the LPF to smooth out response and prevent the mbot from kicking up every time
    float filtered_sp = SET_SPEED;//rc_filter_march(input_f,SET_SPEED);

    // Calculate the error between filtered speed and measured velocity [m/s]
    float error = filtered_sp-MEASURED_SPEED;

    // Computes the controller effort using feedforward open loop control and PID control
    float controller_effort = open_loop_control(MOTOR_CHANNEL,filtered_sp) + rc_filter_march(pid,error) + params.ki*rc_filter_march(integrator,error); // + rc_filter_march(integrator,error);

    return controller_effort;
}

/**
 * @brief Fuses the gyro reading as well as the odometry heading
 * SIMPLE IMPLEMENTATION
 *
 * @param MPU_HEADING
 * @param ODOM_HEADING
 * 
 */
float gyrodometry(float MPU_HEADING, float ODOM_HEADING, float THRESHOLD, rc_filter_t lp, rc_filter_t hp){
    // float new_heading;
    // float blend = 0.5;
    // new_heading = rc_filter_march(&lp,ODOM_HEADING)*blend + rc_filter_march(&hp,MPU_HEADING)*(1-blend);

    // return new_heading;

    if(fabs(MPU_HEADING - ODOM_HEADING) < THRESHOLD){
        return ODOM_HEADING;
    }
    else{
        return MPU_HEADING;
    }
}

/**
 * @brief wheel odometry IMU fusion
 * Takes in odometry and IMU data, returns robot heading
 *
 * @param imu
 * @param odometry
 * 
 */
// float odometry_imu_fusion (mbot_imu_t *imu, odometry_t *odometry, rc_filter_t *lpf, rc_kalman_t *kf){
    
//     // low pass filter for imu data
//     mbot_imu_t imu_filtered;
//     imu_filtered.accel[0] = rc_filter_march(&lpf, imu.accel[0]);
//     imu_filtered.accel[1] = rc_filter_march(&lpf, imu.accel[1]);
//     imu_filtered.accel[2] = rc_filter_march(&lpf, imu.accel[2]);
//     imu_filtered.gyro[0] = rc_filter_march(&lpf, imu.gyro[0]);
//     imu_filtered.gyro[1] = rc_filter_march(&lpf, imu.gyro[1]);
//     imu_filtered.gyro[2] = rc_filter_march(&lpf, imu.gyro[2]);

//     // 

//     // pass both imu_filter and odemetry to kalman filter
//     /******************
//      * TODO: set the values for kalman filter
//     ******************/
//     // while running, measure sensors, calculate y
    
//     // update kalman filter
//     rc_kalman_update_lin(&kf, u, y);
//     // calculate new actuator control output u
    
//     // save u for next loop


//     rc_kalman_free(&kf);

//     float theta_fusion;
//     return theta_fusion;
// }