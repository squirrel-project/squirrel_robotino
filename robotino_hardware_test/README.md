<a id="top"/> 
#robotino_hardware_test

Technical Maintainer: [ipa-nhg](https://github.com/ipa-nhg/) (Nadia Hammoudeh Garcia, Fraunhofer IPA) - nadia.hammoudeh.garcia@ipa.fraunhofer.de

Build status: [Travis Build Status] (https://magnum.travis-ci.com/squirrel-project/squirrel_robotino)

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####Squirrel packages
This repository requires the repositories [squirrel_common](https://github.com/squirrel-project/squirrel_common), and the private ones *squirrel_kclhand* and *squirrel_driver*, in case you don't have access to our private repostitories you can clone [squirrel_substitute](https://github.com/squirrel-project/squirrel_substitute)

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_robotino -i -y
```
## 2. Execution: <a id="2--execution"/> 
###Execute launch file:
```
roslaunch robotino_hardware_test robot.launch robot:='robot_name' result_dir:='/tmp'
```
By default the result will be saved on /tmp file.
On the folder robotino_hardware_config/config/'robot_name'/ you can add new sensors or actuators to be tested.
Available robots:

* tuw-robotino2: robotino base + pan/tilt axis + arm
* uibk-robotino2: robotino base + pan/tilt axis + arm


<a href="#top">top</a>
