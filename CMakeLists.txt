cmake_minimum_required(VERSION 2.8.3)
project(slam)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    eigen_conversions
    rospy
    std_msgs 
    genmsg 
    geometry_msgs 
    fssim_common
    sensor_msgs
    )
catkin_python_setup()

## Declare ROS messages and services


## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs fssim_common sensor_msgs)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
        eigen_conversions
DEPENDS
    ${EIGEN_PACKAGE}
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(slamNode src/cppslam/slamNode.cpp)

add_library(matrix src/cppslam/matrix/matrix.cpp)
add_library(matrixUtils src/cppslam/matrix/matrixUtils.cpp)
add_library(ekf src/cppslam/ekfslam/ekfslam.cpp)

target_link_libraries(slamNode ${catkin_LIBRARIES} matrix matrixUtils ekf)

