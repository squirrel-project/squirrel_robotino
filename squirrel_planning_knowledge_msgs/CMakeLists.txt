cmake_minimum_required(VERSION 2.8.3)
project(squirrel_planning_knowledge_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  diagnostic_msgs
  sensor_msgs
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddObjectService.srv
  UpdateObjectService.srv
  RemoveObjectService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   diagnostic_msgs
   sensor_msgs
   std_msgs
)

## Declare a catkin package
catkin_package(
   CATKIN_DEPENDS 
	message_runtime 
	std_msgs 
	sensor_msgs
	diagnostic_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
