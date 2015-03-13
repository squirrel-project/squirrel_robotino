Start the Robotino Simulation
=============================

```
  roslaunch robotino_bringup_sim robot.launch robot:=_ROBOT_NAME_
```
where _ROBOT_NAME_=ipa-robotino,tuw-robotino...
Start Rviz 
===========
```
  roslaunch robotino_bringup rviz.launch
```

Spawn objects
=============

After you started the gazebo simulation environment you can add objects from robotino_gazebo_worlds to the simulation. Their names and positions for a specific environment are defined in robotino_gazebo_worlds/config/[environment_name]/object_locations.yaml. 

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

Create new objects
==================

### Modelling ###

In general: avoid zero thickness for planes, this will cause rendering problems 

#### Model the object using urdf ####
see: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch. 
    
The currently available colors are the following: http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials if you need a new one, you have to define it in the custom.material file: robotino_gazebo_worlds/Media/materials/scripts/custom.material .

#### Model the object using a mesh model(Collada for example) ####
Put the resulting mesh file into the package robotino_gazebo_worlds into the folder Media/models/. The folder with the textures should also be added to this folder.

Create a new object file in the package robotino_gazebo_worlds in the folder objects named "MY_OBJECT.urdf". You can use the following template and replace the filenames in line tags <mesh filename.../>. Note the scaling in order to reduce the model to metric values.

```
    <?xml version="1.0" ?> 
    <robot name="my_object" static="true">
      <link name="base_link">
      <inertial>
        <mass value="1.0" />
        <origin xyz="0 0 0.0" />
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
      <visual>
       <origin xyz="0 0 0.0" rpy="0 0 0" />
       <geometry>
         <mesh filename="package://robotino_gazebo_worlds/Media/models/my_object.dae"/>
       </geometry>
      </visual>
      <collision>
       <origin xyz="0 0 0" rpy="0 0 0" />
       <geometry>
         <mesh filename="package://robotino_gazebo_worlds/Media/models/my_object.dae"/>
       </geometry>
      </collision>
      </link>

    <gazebo>
	    <static>true</static>
    </gazebo>
    </robot>

```
### Define a object position ###

 You have to define a position for this object in the package cob_gazebo_objects, for each world there a configuration file in: robotino_gazebo_objects/config/[environment_name]/object_locations.yaml. The format for this file is: 

 ```
 MY_OBJECT:
  model: MY_OBJECT
  model_type: urdf
  position: [X,Y,Z]
  orientation: [0, 0, 0]
  ```
### Spawn the new object  ###

Spawn your object using the node spawn object: 

```
  rosrun robotino_bringup_sim spawn_object.py  MY_OBJECT
```
