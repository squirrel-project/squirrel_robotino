cmake_minimum_required(VERSION 2.8.3)
project(robotino_hardware_test)

find_package(catkin REQUIRED COMPONENTS message_generation)

catkin_python_setup()

add_service_files(
  FILES
  Dialog.srv
  TestTrigger.srv
)

generate_messages()

catkin_package()

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS 
  scripts/hardware_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
