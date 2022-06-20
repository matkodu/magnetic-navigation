cmake_minimum_required(VERSION 2.8.3)
project(rosbag_sub_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
 tf
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosbag_sub_pub src/rosbag_sub_pub.cpp)

target_link_libraries(rosbag_sub_pub
  ${catkin_LIBRARIES}
)

