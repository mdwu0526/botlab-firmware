#MBOT
add_executable(mbot
  mbot.c
)
target_link_libraries(mbot
  pico_stdlib
  pico_multicore
  pico_time
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)
pico_enable_stdio_usb(mbot 1)
pico_enable_stdio_uart(mbot 0)
pico_add_extra_outputs(mbot)
