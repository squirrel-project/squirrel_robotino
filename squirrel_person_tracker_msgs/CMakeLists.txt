cmake_minimum_required(VERSION 2.8.3)
project(squirrel_person_tracker_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)


 add_message_files(DIRECTORY msg
   FILES
   State.msg
   HeadHandPoints.msg
   
 )


 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)


