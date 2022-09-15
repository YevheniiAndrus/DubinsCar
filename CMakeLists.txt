cmake_minimum_required(VERSION 3.10)

project( DubinsCar )
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(car)

set(HEADERS simulation.hpp)
set(SOURCE simulation.cpp)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( DubinsCar ${SOURCE} ${HEADERS} main.cpp )
target_link_libraries( DubinsCar  car ${OpenCV_LIBS} )