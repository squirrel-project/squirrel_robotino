<a id="top"/> 
#squirrel_robotino

This repository holds packages for hardware launch files and configuration, as well as the simulation model for starting up the basic layer for operating Robotino

Technical Maintainer: [ipa-nhg](https://github.com/ipa-nhg/) (Nadia Hammoudeh Garcia, Fraunhofer IPA) - nadia.hammoudeh.garcia@ipa.fraunhofer.de

Build status: [Travis Build Status] (https://magnum.travis-ci.com/squirrel-project/squirrel_robotino)

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>
4. <a href="#4--dynamixel-servos">Dynamixel servos</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####Debian packages
The robotino-api2 has to be installed to compile the robotino_driver package: 
```
echo "deb http://doc.openrobotino.org/download/packages/amd64 ./" >> /etc/apt/sources.list
sudo apt-get update 
sudo apt-get install robotino-api2
```
####Squirrel packages
This repository requires the repositories [squirrel_common](https://github.com/squirrel-project/squirrel_common), and the private ones *squirrel_kclhand* and *squirrel_driver*, in case you don't have access to our private repostitories you can clone [squirrel_substitute](https://github.com/squirrel-project/squirrel_substitute)

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_robotino -i -y
```
## 2. Execution: <a id="2--execution"/> 
###Real robot:
```
roslaunch robotino_bringup robot.launch robot:='robot_name'
```
###Simulation:
```
roslaunch robotino_bringup_sim robot.launch robot:='robot_name'
```
Available robots:

* alufr-robotino: robotino base + tilt axis
* ipa-robotino: robotino base + tilt axis
* robotino: robotino base + arm
* tuw-robotino: robotino base + pan/tilt axis
* tuw-robotino2: robotino base + pan/tilt axis + arm + hand
* uibk-robotino: robotino base + pan/tilt axis
* uibk-robotino2: robotino base + pan/tilt axis + arm + hand

## 3. Software architecture <a id="3--software-architecture"/> 

robotino_node: ![robotino_node](https://github.com/squirrel-project/squirrel_robotino/blob/indigo_dev/robotino_node.png "Architecture")

robotino_simulation: ![robotino_simulation](https://github.com/squirrel-project/squirrel_robotino/blob/indigo_dev/squirrel_simulation.png "Architecture")

## 4. Dynamixel servos <a id="4--dynamixel-servos"/> 

We are using Dynamixel servos in the pan/tilt unit. The ROS package for these is `dynamixel_driver`. To see if servos are connected, powered and generally ok, call `info_dump` with a specific baud rate (`57142`) and with the servo IDs, which should be 1 and 2. If you do not find the servos, you can try up to 254.
```
> rosrun dynamixel_driver info_dump.py -b 57142 1 2
Pinging motors:
1 ... done
    Motor 1 is connected:
        Freespin: False
        Model ------------------- MX-28 (firmware version: 36)
        Min Angle --------------- 0
        Max Angle --------------- 4095
        Current Position -------- 65
        Current Speed ----------- 0
        Current Temperature ----- 33°C
        Current Voltage --------- 11.7v
        Current Load ------------ 0
        Moving ------------------ False
...
```
If you get a new servo, its ID will be set to 1. Use `change_id`, e.g. to 2:
```
> rosrun dynamixel_driver change_id.py -b 57142 1 2
Changing motor id from 1 to 2...  done
Verifying new id... ERROR: The motor did not respond to a ping to its new id.
```
Never mind the error. Just call `info_dump` again to see that the servo now is set correctly.

<a href="#top">top</a>
