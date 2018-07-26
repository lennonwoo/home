cmake_minimum_required(VERSION 2.8.3)
project(home)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  facenet_ros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
