cmake_minimum_required(VERSION 2.8)
project(state_transform)

# set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    quadrotor_msgs
    airsim_ros_pkgs
    mavros
    uav_utils
    tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS})

link_directories(
    ${PCL_LIBRARY_DIRS})

add_executable (st_node 
    src/state_transform.cpp)

target_link_libraries(st_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})



