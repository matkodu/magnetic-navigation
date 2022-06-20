cmake_minimum_required(VERSION 2.8.3)
project(orientation)

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

add_executable(orientation src/orientation.cpp)

target_link_libraries(orientation
  ${catkin_LIBRARIES}
)

