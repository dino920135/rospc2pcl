cmake_minimum_required(VERSION 3.10)
project(load_RSCan)

file(GLOB SOURCES "src/*.cpp")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    sensor_msgs
    geometry_msgs
    tf
)

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(load_RScan ${SOURCES})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ./include)
target_link_libraries(load_RScan ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_INCLUDE_DIRS})