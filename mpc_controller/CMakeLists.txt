cmake_minimum_required(VERSION 3.0.2)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()
set(CMAKE_CXX_FLAGS "-g -O0")

project(mpc_controller)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -g")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address,undefined")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

message(STATUS ${OSQP_INCLUDE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS} 
                    ${OSQP_INCLUDE_DIRS})



cs_add_executable(${PROJECT_NAME}
src/control_v2.cpp
src/mpc_control.cpp)

target_link_libraries(${PROJECT_NAME}
${Eigen3_INCLUDE_DIRS}
osqp::osqp)

cs_install()
cs_export()