#!/bin/sh
set -e
set -v

wget http://doc.openrobotino.org/download/packages/amd64/robotino-api2_0.9.16_amd64.deb -O /tmp/robotino.deb
dpkg -i /tmp/robotino.deb

#MYARCH=$(uname -m)
#apt-get install dpkg
#dpkg -i $CATKIN_WORKSPACE/src/squirrel_driver/ft17_driver/ft17driver_0.0-1_${MYARCH}.deb
