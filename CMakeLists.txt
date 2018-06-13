cmake_minimum_required(VERSION 2.8.3)
project(glc-lib)

##Compiler flags for release and debug modes
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug

##Include directory for glc headers
set(GLC_INCLUDES ${CMAKE_SOURCE_DIR}/include)
include_directories(${GLC_INCLUDES})

##Source files for demos
set(DEMO_SOURCE ${CMAKE_SOURCE_DIR}/src)

##Put the executables in the examples folder
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/examples)

add_executable(shortest-path ${DEMO_SOURCE}/shortest-path.cpp)
add_executable(car-demo ${DEMO_SOURCE}/nonholonomic-car-demo.cpp)
