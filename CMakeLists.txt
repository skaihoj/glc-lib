cmake_minimum_required(VERSION 3.5)
project(glc-lib)

#find_package(GTest REQUIRED)
enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -std=c++0x -fopenmp -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x")#for debug

include_directories(${CMAKE_SOURCE_DIR}/include)
#include_directories(${GTEST_INCLUDE_DIRS})

set(SRC_PATH ${CMAKE_SOURCE_DIR}/src/glc)
set(EXAMPLE_PATH ${CMAKE_SOURCE_DIR}/src)
set(TEST_PATH ${CMAKE_SOURCE_DIR}/test)

#Level-0 of local dependency graph
add_library(glc_math ${SRC_PATH}/glc_math.cpp)
add_library(glc_interpolation ${SRC_PATH}/glc_interpolation.cpp)
add_library(glc_node ${SRC_PATH}/glc_node.cpp)
add_library(glc_parameters ${SRC_PATH}/glc_parameters.cpp)

#Level-1 of local dependency graph
add_library(glc_state_equivalence_class ${SRC_PATH}/glc_state_equivalence_class.cpp)
target_link_libraries(glc_state_equivalence_class
                      glc_node
                      glc_math)

add_library(glc_interface ${SRC_PATH}/glc_interface.cpp)
target_link_libraries(glc_interface
                      glc_interpolation)
                      
#Level-2 of local dependency graph
add_library(glc_logging ${SRC_PATH}/glc_logging.cpp)
target_link_libraries(glc_logging 
                      glc_interpolation
                      glc_state_equivalence_class)
                      
add_library(glc_numerical_integration ${SRC_PATH}/glc_numerical_integration.cpp)
target_link_libraries(glc_numerical_integration
                      glc_interpolation
                      glc_interface)

#Level-3 of local dependency graph                      
add_library(glc_planner_core ${SRC_PATH}/glc_planner_core.cpp)
target_link_libraries(glc_planner_core
                      glc_numerical_integration
                      glc_logging)

##Put the executables in the examples folder
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build/bin)

# add_executable(shortest-path ${DEMO_SOURCE}/shortest-path.cpp)
# add_executable(car-demo ${DEMO_SOURCE}/nonholonomic-car-demo.cpp)

#Unit Test
add_executable(test_glc_math ${TEST_PATH}/test_glc_math.cpp)
target_link_libraries(test_glc_math
                      #${GTEST_BOTH_LIBRARIES} 
                      glc_math)
#add_test(gtest_glc_math test_glc_math)

#System Test
add_executable(base-test ${TEST_PATH}/base_test.cpp)
target_link_libraries(base-test 
                      ${GTEST_BOTH_LIBRARIES} 
                      glc_planner_core)
                      
#add_test(gtest-base-test base-test)

## Installation ##

set(INCLUDE_INSTALL_ROOT_DIR include)
set(INCLUDE_INSTALL_DIR ${INCLUDE_INSTALL_ROOT_DIR}/glc)
set(LIB_INSTALL_DIR "lib${LIB_SUFFIX}")

set(_INSTALL_DESTINATIONS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
)

install(TARGETS glc_planner_core
                glc_interface
                glc_interpolation
                glc_logging
                glc_math
                glc_state_equivalence_class
        EXPORT  
                ${_INSTALL_DESTINATIONS})
install(
  DIRECTORY ${header_directory}
  DESTINATION ${INCLUDE_INSTALL_DIR}
  FILES_MATCHING PATTERN "*.h"
)
