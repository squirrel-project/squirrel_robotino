Start the Robotino Simulation
=============================

```
  roslaunch robotino_bringup_sim robot.launch
```

Start Rviz 
===========
```
  roslaunch robotino_bringup rviz.launch
```

Spawn objects
=============

After you started the gazebo simulation environment you can add objects from robotino_gazebo_worlds to the simulation. Their names and positions for a specific environment are defined in robtino_gazebo_worlds/config/[environment_name]/object_locations.yaml. 

To add objects you can use
```
  rosrun robotino_bringup_sim spawn_object.py OBJECT_NAME1 OBJECT_NAME2 OBJECT_NAME3 ...
```
or
```
  rosrun robotino_bringup_sim spawn_object.py all
```
There are pre-defined positions for the objects depending on the environment they get spawned.

You can remove objects with
```
  rosrun robotino_bringup_sim remove_object.py OBJECT_NAME1 OBJECT_NAME2 OBJECT_NAME3 ...
```
or
```
  rosrun robotino_bringup_sim remove_object.py all
```

