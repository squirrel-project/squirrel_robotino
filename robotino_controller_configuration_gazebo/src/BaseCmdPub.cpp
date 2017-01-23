/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: SQUIRREL
 * ROS stack name: squirrel_robotino
 * ROS package name: robotino_controller_configuration_gazebo
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fraunhofer.de
 * Supervised by:  Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fraunhofer.de
 *
 * Date of creation: September 2016
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_sim_pub");
  ros::NodeHandle nh_;
  double dt= 0.01;
  geometry_msgs::Twist cmdVel_;

  ros::Publisher cmdPub = nh_.advertise<geometry_msgs::Twist>("cmd_sim", 1/dt);

  cmdVel_.linear.x = 0;
  cmdVel_.linear.y = 0;
  cmdVel_.angular.z = 0;
  ros::Rate loop_rate(10);

  while (ros::ok()){
	  cmdPub.publish(cmdVel_);
    loop_rate.sleep();
	  ros::spinOnce();
  }
  return 0;

}

