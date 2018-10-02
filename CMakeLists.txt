cmake_minimum_required(VERSION 3.5)
project(glc-lib)

find_package(GTest REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -fopenmp -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${GTEST_INCLUDE_DIRS})

set(SRC_PATH ${CMAKE_SOURCE_DIR}/src/glc)
set(EXAMPLE_PATH ${CMAKE_SOURCE_DIR}/src)
set(TEST_PATH ${CMAKE_SOURCE_DIR}/test)

#Level-0 of dependancy graph
add_library(glc_math ${SRC_PATH}/glc_math.cpp)
add_library(glc_interpolation ${SRC_PATH}/glc_interpolation.cpp)
add_library(glc_node ${SRC_PATH}/glc_node.cpp)

#Level-1 of dependancy graph
add_library(glc_state_equivalence_class ${SRC_PATH}/glc_state_equivalence_class.cpp)
target_link_libraries(glc_state_equivalence_class
                      glc_node
                      glc_math)



##Put the executables in the examples folder
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build/bin)

# add_executable(shortest-path ${DEMO_SOURCE}/shortest-path.cpp)
# add_executable(car-demo ${DEMO_SOURCE}/nonholonomic-car-demo.cpp)

add_executable(base-test ${TEST_PATH}/base_test.cpp)
target_link_libraries(base-test 
                      ${GTEST_BOTH_LIBRARIES} 
                      glc_math 
                      glc_interpolation 
                      glc_node
                      glc_state_equivalence_class)
                      
add_test(gtest-base-test base-test)