cmake_minimum_required(VERSION 2.8.3)
project(robotino_gazebo_worlds)


find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS gazebo_ros kdl_parser
)


install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

