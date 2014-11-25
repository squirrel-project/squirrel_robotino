cmake_minimum_required(VERSION 2.8.3)
project(robotino_controller_configuration_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package(
  CATKIN_DEPENDS
)


## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY controller
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/fake_diagnostics.py src/gazebo_services.py src/gazebo_topics.py src/gazebo_services_base.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
