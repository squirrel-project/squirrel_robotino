#!/bin/bash

export MY_CATKIN_WORKSPACE=~/catkin_ws

. /opt/ros/indigo/setup.sh

if [ -e $MY_CATKIN_WORKSPACE/devel/setup.bash ]; then
    source $MY_CATKIN_WORKSPACE/devel/setup.bash
elif [ -e /u/squirrel/catkin_ws/devel/setup.bash ]; then
    source /u/squirrel/catkin_ws/devel/setup.bash
else
    source /opt/ros/indigo/setup.bash
fi

export ROS_IP=`hostname -I | awk '{print $1}'`

exec "$@"
