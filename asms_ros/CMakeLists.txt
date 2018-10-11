cmake_minimum_required(VERSION 2.8.3)
project(asms_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wextra -pedantic -Wno-long-long")
endif()

include_directories(src)
add_subdirectory(src)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
  message_filters
)

add_message_files(
  FILES
  BoundingBox.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(asms_demo src/main_demo.cc)
#target_link_libraries(asms_demo ${OpenCV_LIBRARIES} ${OpenCV_LIBS} color)

add_executable(asms_node src/asms_node.cc src/asms.cc)
target_link_libraries(asms_node color ${catkin_LIBRARIES})
