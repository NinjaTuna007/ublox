cmake_minimum_required(VERSION 3.22)
project(ublox_stick_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/stick_bringup.sh
    scripts/install_ublox_udev.sh
    scripts/sound_velocity_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
