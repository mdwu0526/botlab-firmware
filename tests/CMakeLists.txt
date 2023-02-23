
#pico_dmp_test
add_executable(pico_dmp_test
  pico_dmp_test.c
)
target_link_libraries(pico_dmp_test
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(pico_dmp_test 1)
pico_enable_stdio_uart(pico_dmp_test 0)
pico_add_extra_outputs(pico_dmp_test)

#pico_calibrate_accel
add_executable(pico_calibrate_accel
  pico_calibrate_accel.c
)
target_link_libraries(pico_calibrate_accel
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(pico_calibrate_accel 1)
pico_enable_stdio_uart(pico_calibrate_accel 0)
pico_add_extra_outputs(pico_calibrate_accel)

#pico_calibrate_gyro
add_executable(pico_calibrate_gyro
  pico_calibrate_gyro.c
)
target_link_libraries(pico_calibrate_gyro
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(pico_calibrate_gyro 1)
pico_enable_stdio_uart(pico_calibrate_gyro 0)
pico_add_extra_outputs(pico_calibrate_gyro)

#MATRIX-TESTS
add_executable(pico_matrix_test
pico_matrix_test.c
)
target_link_libraries(pico_matrix_test
  pico_stdlib
  rclib
)
pico_enable_stdio_usb(pico_matrix_test 1)
pico_add_extra_outputs(pico_matrix_test)


#FRAM-TEST
add_executable(pico_fram_test
  pico_fram_test.c
)
target_link_libraries(pico_fram_test
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(pico_fram_test 1)
pico_enable_stdio_uart(pico_fram_test 0)
pico_add_extra_outputs(pico_fram_test)

#MOTOR-TEST
add_executable(pico_motor_test
  pico_motor_test.c
)
target_link_libraries(pico_motor_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(pico_motor_test 1)
pico_enable_stdio_uart(pico_motor_test 0)
pico_add_extra_outputs(pico_motor_test)

#MOTOR-ANALYZE
add_executable(pico_motor_analyze
  pico_motor_analyze.c
)
target_link_libraries(pico_motor_analyze
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(pico_motor_analyze 1)
pico_enable_stdio_uart(pico_motor_analyze 0)
pico_add_extra_outputs(pico_motor_analyze)

#I control tests
add_executable(pico_current_control_test
  pico_current_control_test.c
)
target_link_libraries(pico_current_control_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(pico_current_control_test 1)
pico_enable_stdio_uart(pico_current_control_test 0)
pico_add_extra_outputs(pico_current_control_test)


#Encoder tests
add_executable(pico_encoder_test
  pico_encoder_test.c
)
target_link_libraries(pico_encoder_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(pico_encoder_test 1)
pico_enable_stdio_uart(pico_encoder_test 0)
pico_add_extra_outputs(pico_encoder_test)

# Teleop
add_executable(pico_teleop
  pico_teleop.c
)
target_link_libraries(pico_teleop
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
  comms
)
pico_enable_stdio_usb(pico_teleop 1)
pico_enable_stdio_uart(pico_teleop 0)
pico_add_extra_outputs(pico_teleop)