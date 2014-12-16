cmake_minimum_required(VERSION 2.8.3)
project(robotino_controller_configuration_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roslaunch roscpp geometry_msgs std_msgs
)

## Building the executables

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotino_controller_configuration_gazebo_node
  src/RobotinoControllerNode.cpp
  src/RobotinoController.cpp
)

target_link_libraries(robotino_controller_configuration_gazebo_node
  ${catkin_LIBRARIES}
)

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
  robotino_controller_configuration_gazebo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
