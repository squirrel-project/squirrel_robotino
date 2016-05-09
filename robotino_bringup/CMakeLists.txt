cmake_minimum_required(VERSION 2.8.3)
project(robotino_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY robots
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS 
  src/dynamixel_joint_state_publisher.py
  src/cmd_rotatory_startup_locker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
