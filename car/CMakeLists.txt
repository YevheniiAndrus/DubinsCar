cmake_minimum_required(VERSION 3.10)
project(car)
set(HEADERS car.hpp)
set(SOURCE car.cpp)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(car ${SOURCE} ${HEADERS})
target_link_libraries( car ${OpenCV_LIBS} )