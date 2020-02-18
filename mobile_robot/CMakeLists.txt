cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(INCLUDE_DIRS include/mobile_robot
CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs)

include_directories(include/mobile_robot ${catkin_INCLUDE_DIRS})

add_executable(mobile_robot src/main.cpp src/Obstacle.cpp)

target_link_libraries(mobile_robot ${OpenCV_LIBS} ${catkin_LIBRARIES})
