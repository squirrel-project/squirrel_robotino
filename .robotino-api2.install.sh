#!/bin/sh
set -e
set -v

wget http://doc.openrobotino.org/download/packages/amd64/robotino-api2_0.9.16_amd64.deb -O /tmp/robotino.deb
dpkg -i /tmp/robotino.deb
