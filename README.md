squirrel_robotino
=================
[![Build Status](https://magnum.travis-ci.com/squirrel-project/squirrel_robotino.svg?token=3yXoCRsCegowgzzpPuqw)](https://magnum.travis-ci.com/squirrel-project/squirrel_robotino)

Technical Maintainer: ipa-nhg (Nadia Hammoudeh Garcia, Fraunhofer IPA)

This repository holds packages for hardware launch files and configuration, as well as the simulation model for starting up the basic layer for operating Robotino

It requires the installation of the robotino-api2 packages:
```
wget -qO - http://packages.openrobotino.org/keyFile | sudo apt-key add -
sudo su
echo "deb http://packages.openrobotino.org/trusty trusty main" > /etc/apt/sources.list.d/openrobotino.list
apt-get update
apt-get install rec-rpc robotino-common robotino_daemons robotino-api2 robotino-examples robotino3-firmware
```
And install the package dependencies:
```
rosdep install --from-path squirrel_robotino -i -y
```

# Dynamixel servos
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
