cmake_minimum_required(VERSION 3.10)
project(load_RSCan)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    sensor_msgs
    geometry_msgs
    tf
)

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(load_RScan load_radar_point.cpp)
target_link_libraries(load_RScan ${catkin_LIBRARIES} ${PCL_LIBRARIES})