cmake_minimum_required(VERSION 2.8.3)
project(magnetic_sensors_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
  std_msgs
  message_generation
)


catkin_package(
  CATKIN_DEPENDS
    serial
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(magnetic_sensors_arduino src/magnetic_sensors_arduino.cpp)



target_link_libraries(magnetic_sensors_arduino
  ${catkin_LIBRARIES}
)

