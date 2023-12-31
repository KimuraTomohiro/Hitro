cmake_minimum_required(VERSION 3.0.2)
project(Hitro_move)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
dynamixel_workbench_toolbox
roscpp
rospy
std_msgs
sensor_msgs
geometry_msgs
message_generation
ps5controller
# moveit_core
# moveit_ros_planning
# moveit_ros_planning_interface
rviz
arm_msgs
motor_msgs
cv_bridge
)
set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)

set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  wheelstatus.msg
  ArmCommand.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
   )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES Hitro_move
#  CATKIN_DEPENDS message_generation roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(Hitro_move
  src/dynamixel_main.cpp 
  src/dynamixel_class.cpp
  )
target_link_libraries(Hitro_move
${catkin_LIBRARIES}
)

add_executable(arm_dxl
  src/arm_dxl_main.cpp 
  src/arm_dxl_class.cpp
  )
target_link_libraries(arm_dxl
${catkin_LIBRARIES}
)

add_executable(position_loader
  src/position_loader_main.cpp 
  src/position_loader_class.cpp
  )
target_link_libraries(position_loader
${catkin_LIBRARIES}
)
# add_executable(kamuy_arm_gui src/kamuy_arm_gui.cpp)

# target_link_libraries(kamuy_arm_gui
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_library(${PROJECT_NAME} ${SRC_FILES})
# add_library(${PROJECT_NAME}
#  src/km_gui_dxl.cpp
#  )

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${QT_LIBRARIES}
# )
# add_executable(arm_position_loader
#   src/arm_pl_main.cpp 
#   src/arm_pl_class.cpp
#   )
# target_link_libraries(position_loader
# ${catkin_LIBRARIES}
# )
# add_executable(getposition
#   src/get_position.cpp 
#   src/dynamixel_class.cpp
#   )
# target_link_libraries(getposition
# ${catkin_LIBRARIES}
# )


# add_executable(ps5toTwist
#   src/joy_subscriber.cpp 
#   )
# target_link_libraries(ps5toTwist
# ${catkin_LIBRARIES}
# )



# add_executable(dog_target_generator
#   src/dog_target_generator.cpp 
#   )
# target_link_libraries(dog_target_generator
# ${catkin_LIBRARIES}
# )


add_executable(ps5toTwist
  src/joy_subscriber.cpp 
  )
target_link_libraries(ps5toTwist
${catkin_LIBRARIES}
)

add_executable(ps5toTwist_hp
  src/joy_subscriber_hp.cpp 
  )
target_link_libraries(ps5toTwist_hp
${catkin_LIBRARIES}
)

add_executable(urdf2dxl
  src/urdf2dxl.cpp 
  )
target_link_libraries(urdf2dxl
${catkin_LIBRARIES}
)

add_executable(Hitro_arm
  src/Hitro_arm.cpp 
  )
target_link_libraries(Hitro_arm
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)