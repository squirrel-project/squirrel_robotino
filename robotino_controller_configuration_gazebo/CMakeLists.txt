cmake_minimum_required(VERSION 2.8.3)
project(robotino_controller_configuration_gazebo)

find_package(catkin REQUIRED COMPONENTS
	roslaunch
	roscpp
	geometry_msgs
	std_msgs
	velocity_controllers
	controller_manager
	robot_state_publisher
	topic_tools
	gazebo_ros_control
	rostopic
	joint_state_controller
	robotino_bringup
	effort_controllers
	tf
	nav_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS 
	roslaunch
	roscpp
	geometry_msgs
	std_msgs
	velocity_controllers
	controller_manager
	robot_state_publisher
	topic_tools
	gazebo_ros_control
	rostopic
	joint_state_controller
	robotino_bringup
	effort_controllers
	tf
	nav_msgs
)

## Building the executables

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(base_sim_controller
  src/RobotinoControllerNode.cpp
  src/RobotinoController.cpp
)

add_executable(base_sim_odometry
  src/OdometryController.cpp
)

add_executable(base_cmd_sim src/BaseCmdPub.cpp)
add_dependencies(base_cmd_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_cmd_sim ${catkin_LIBRARIES})


#add_executable(arm_sim_controller
#  src/ArmController.cpp
#)

#add_executable(rotatory_sim_control
#src/RotatorySimControl.cpp)

target_link_libraries(base_sim_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(base_sim_odometry
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


#target_link_libraries(arm_sim_controller
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#target_link_libraries(rotatory_sim_control
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

## Mark cpp header files for installation

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY controller
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
