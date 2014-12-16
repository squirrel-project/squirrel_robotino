#include "robotino_controller_configuration_gazebo/RobotinoController.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robotino_controller_gazebo");
  robotino_controller_configuration_gazebo::RobotinoController rc;
  rc.spin();
  return 0;
}
