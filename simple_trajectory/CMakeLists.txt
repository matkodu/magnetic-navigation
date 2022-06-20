cmake_minimum_required(VERSION 2.8.3)
project(simple_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  rospy
 tf
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
)
#catkin_install_python(PROGRAMS src/simple_trajectory_py.py
 #                     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_trajectory src/simple_trajectory.cpp)

target_link_libraries(simple_trajectory
  ${catkin_LIBRARIES}
)

