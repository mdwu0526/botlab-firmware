
#TEST_DMP
add_executable(test_dmp
  test_dmp.c
)
target_link_libraries(test_dmp
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(test_dmp 1)
pico_enable_stdio_uart(test_dmp 0)
pico_add_extra_outputs(test_dmp)

#MATRIX-TESTS
add_executable(matrix-tests
matrix-tests.c
)
target_link_libraries(matrix-tests
  pico_stdlib
  rclib
)
pico_enable_stdio_usb(matrix-tests 1)
pico_add_extra_outputs(matrix-tests)


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

#DRIVE_FORWARD
add_executable(pico_drive_forward
  pico_drive_forward.c
)
target_link_libraries(pico_drive_forward
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(pico_drive_forward 1)
pico_enable_stdio_uart(pico_drive_forward 0)
pico_add_extra_outputs(pico_drive_forward)

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
add_executable(pico_current_control
  pico_current_control.c
)
target_link_libraries(pico_current_control
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)
pico_enable_stdio_usb(pico_current_control 1)
pico_enable_stdio_uart(pico_current_control 0)
pico_add_extra_outputs(pico_current_control)


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

#Encoder output
add_executable(pico_encoder_output
  pico_encoder_output.c
)
target_link_libraries(pico_encoder_output
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(pico_encoder_output 1)
pico_enable_stdio_uart(pico_encoder_output 0)
pico_add_extra_outputs(pico_encoder_output)

#Drive robot
add_executable(test_driving
  test_driving.c
)
target_link_libraries(test_driving
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)
pico_enable_stdio_usb(test_driving 1)
pico_enable_stdio_uart(test_driving 0)
pico_add_extra_outputs(test_driving)