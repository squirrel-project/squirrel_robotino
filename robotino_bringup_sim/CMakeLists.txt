# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotino_bringup_sim)

find_package(catkin REQUIRED COMPONENTS)


catkin_package(
    CATKIN_DEPENDS gazebo_ros robotino_gazebo_worlds
    DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/remove_object.py scripts/spawn_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
