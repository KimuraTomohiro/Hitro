cmake_minimum_required(VERSION 3.0.2)
project(Hitro_Crawler_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  flipper_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Hitro_Crawler_controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(base_controller src/base_controller.cpp)
add_dependencies(base_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_controller
  ${catkin_LIBRARIES}
)