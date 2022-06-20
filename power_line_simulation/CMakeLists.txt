cmake_minimum_required(VERSION 2.8.3)
project(power_line_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
 tf
serial
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(power_line_simulation src/power_line_simulation.cpp)

target_link_libraries(power_line_simulation
  ${catkin_LIBRARIES}
)

