squirrel_robotino
=================
[![Build Status](https://magnum.travis-ci.com/squirrel-project/squirrel_robotino.svg?token=3yXoCRsCegowgzzpPuqw)](https://magnum.travis-ci.com/squirrel-project/squirrel_robotino)

Technical Maintainer: ipa-nhg (Nadia Hammoudeh Garcia, Fraunhofer IPA)

This repository holds packages for hardware launch files and configuration, as well as the simulation model for starting up the basic layer for operating Robotino

#####Hydro

Install the package dependencies:
```
rosdep install --from-path squirrel_robotino -i -y
```
#####Indigo

To use the simulator, please download the following repositories into your catkin workspace:
```
git clone https://github.com/ipa320/gazebo_ros_pkgs.git
git clone https://github.com/ipa320/ros_controllers.git
git clone https://github.com/ipa320/ros_control.git
git clone https://github.com/ipa320/cob_gazebo_plugins.git 
```
This cloning should be replaced by installing the respective packages with sudo apt-get install by January 2015.

