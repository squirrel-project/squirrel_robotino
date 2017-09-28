#!/bin/bash

export MY_CATKIN_WORKSPACE=~/catkin_ws

. /opt/ros/indigo/setup.sh

SOURCE /home/pi/catkin_ws/devel/setup.bash

export ROS_IP=`hostname -I | awk '{print $1}'`

exec "$@"
