cmake_minimum_required(VERSION 2.8.3)
project(slam)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
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



## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(slam src/cppslam/slamNode.cpp)

add_library(matrix src/cppslam/matrix/matrix.cpp)
add_library(matrixUtils src/cppslam/matrix/matrixUtils.cpp)
add_library(ekf src/cppslam/ekfslam/ekf.cpp)

target_link_libraries(slam ${catkin_LIBRARIES} matrix matrixUtils ekf)

