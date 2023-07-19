
# cmake_minimum_required(VERSION 3.0.2)

find_package(OpenCV REQUIRED)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

include_directories(${BORIS_MATH_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(PIDSimulator PIDSimulator.cpp)
target_link_libraries(PIDSimulator ${OpenCV_LIBS} ${BORIS_CONTROL_LIBRARIES})