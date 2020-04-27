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
    message_generation
    sensor_msgs
    mur_common
    )
catkin_python_setup()


find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()
find_package(Boost REQUIRED COMPONENTS thread)


set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall;-Werror")
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
        eigen_conversions
DEPENDS
    ${EIGEN_PACKAGE}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(slamNode src/cppslam/slamNode.cpp)

add_library(ekf src/cppslam/ekfslam/ekfslam.cpp)
add_library(discreteBayes src/cppslam/discreteBayesFilter/discreteBayes.cpp)
add_library(point src/cppslam/point/point.cpp)

target_link_libraries(slamNode ${catkin_LIBRARIES} ekf discreteBayes)

add_dependencies(slamNode ${catkin_EXPORTED_TARGETS})