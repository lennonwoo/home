cmake_minimum_required(VERSION 2.8)

set(COLOR_LIB_SRC colotracker.cc histogram.cc region.cc)

find_package(OpenCV 3 REQUIRED)

add_library(color STATIC ${COLOR_LIB_SRC})
target_link_libraries(color ${OpenCV_LIBS})
set_target_properties(color PROPERTIES VERSION 1.0.0 SOVERSION 1)