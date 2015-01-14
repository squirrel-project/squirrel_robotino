cmake_minimum_required(VERSION 2.8.3)
project(robotino_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
        robotino_driver 
	sensor_msgs
	geometry_msgs
	dynamic_reconfigure
	actionlib
)

catkin_package(
    CATKIN_DEPENDS roscpp robotino_driver sensor_msgs geometry_msgs dynamic_reconfigure actionlib

    DEPENDS # TODO
    INCLUDE_DIRS #include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
add_executable(joystick_teleop_node 
	src/joystick_teleop_node.cpp
	src/JoystickTeleop.cpp)

add_executable(keyboard_teleop_node 
	src/keyboard_teleop_node.cpp
	src/KeyboardTeleop.cpp)

target_link_libraries(keyboard_teleop_node ${catkin_LIBRARIES})
target_link_libraries(joystick_teleop_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



