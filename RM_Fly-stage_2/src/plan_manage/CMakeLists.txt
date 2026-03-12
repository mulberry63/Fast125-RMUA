cmake_minimum_required(VERSION 3.10.0)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(AIRSIM_ROOT ${PROJECT_SOURCE_DIR}/../utils/thirdparty/)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  octomap_ros
  octomap_msgs
  octomap_server
  airsim_ros_pkgs
  circle_detector
)
include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${AIRSIM_ROOT}/AirLib/include
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS OpenCV Eigen Boost
)

add_executable(plan_manage
    src/main.cpp
    src/plan_manage.cpp
    src/global_traj_opt.cc
    src/global_traj_opt_replan.cc
    src/rm_traj_opt_uniform.cc
)
target_link_libraries(plan_manage
  ${catkin_LIBRARIES}
  AirLib
)

add_executable(debug
    src/debug.cpp
    src/plan_manage.cpp
    src/global_traj_opt.cc
    src/global_traj_opt_replan.cc
    src/rm_traj_opt_uniform.cc
)
target_link_libraries(debug
  ${catkin_LIBRARIES}
  AirLib
)

