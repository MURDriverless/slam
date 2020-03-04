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

# To be included if cpp slam is used, currently deleted from repo
# add_executable(slam src/slam.cpp)
# target_link_libraries(slam ${catkin_LIBRARIES})
# add_dependencies(slam slam_cpp)
