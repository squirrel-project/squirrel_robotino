1. Measure laser scanner height and put in the number into the properties.urdf.xacro file of your robot which is located, e.g., at squirrel_robotino/robotino_bringup/robots/uibk-robotino/urdf/properties.urdf.xacro.
There search for the block:
  <!-- hokuyo mount positions | relative to base_link -->
  <property name="hokuyo_x" value="0.131740483"/>
  <property name="hokuyo_y" value="0.00937244242"/>
  <property name="hokuyo_z" value="0.102"/> <!-- not used, apprx -->
  <property name="hokuyo_roll" value="0.0"/>
  <property name="hokuyo_pitch" value="0.0"/>
  <property name="hokuyo_yaw" value="-0.0536545473"/>
and enter the measure at hokuyo_z.

2. Mount a calibration checkerboard horizontally on a wall (use a level to be precise), approximately at the height of Robotino's Kinect. There should not be any object around, try to find a flat wall segment of at least 2.5m length and place robotino and the checkerboard in teh center.
Place a box directly below the leftmost line of checkerboard calibration points (i.e. where 4 squares meet). Align the left side of the box with the leftmost calibration points. The box must stand apart from the wall by more than 11 cm. The box is used to localize the calibration pattern with the laser scanner.
Measure the vertical distance between the laser scanner plane and the upper left checkerboard point and insert the number into file 'squirrel_robotino/robotino_calibration/ros/launch/checkerboard_localisation.launch' at the line:
<node pkg="tf" type="static_transform_publisher" name="static_transform_publisher_checkerboard_reflector" output="screen" args="0.0 -0.7	 0 0 0 0  checkerboard_reference checkerboard 100"/>
Replace the -0.5 at the y-offset with your measured number (all other numbers should be 0). Use a negative measure.

-> enter properties of calibration pattern (grid size, square side length) into file 'squirrel_robotino/robotino_calibration/ros/launch/camera_base_calibration.launch' at the line:



RViz --> Marker Topic /wall_marker displays detected wall (green line) and the box in front of the wall (blue points)


3. Start roslaunch robotino_calibration camera_base_calibration.launch


