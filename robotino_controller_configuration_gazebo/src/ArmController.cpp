/****************************************************************
 *
 * Copyright (c) 2016
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
 * Date of creation: Juli 2016
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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <uibk_robot_driver/base_controller.hpp>

using namespace std;

class ArmController{
public:
  ros::NodeHandle nh_;
  ros::Publisher  cmdVelPub_ ;
  ros::Publisher  ArmPosCommandPub_ ;
  void PosCommandSub_cb(const std_msgs::Float64MultiArray msg);
};

void ArmController::PosCommandSub_cb(const std_msgs::Float64MultiArray msg){
  BaseController robotino(nh_, 20.0, 0.6, 1, 1);

  double target = robotino.getCurrentState();
  vector<double> current_pose = robotino.getCurrentPose();
  double cx = current_pose.at(0);
  double cy = current_pose.at(1);
    

  vector<double> current_pose = robotino.getCurrentPose();
  robotino.move(target+msg.data[0] , cx+0.80, cy+0.80);

  ArmPosCommandPub_ = nh_.advertise<std_msgs::Float64MultiArray>("arm_controller/group_position_controller/cmd", 1);
  std_msgs::Float64MultiArray joint_pos;
  joint_pos.data.resize(msg.data.size()-1);

  for (int i=1; i < msg.data.size(); i++){
    joint_pos.data[i-1] = msg.data[i];
  }
  ArmPosCommandPub_.publish(joint_pos);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "ArmController");
  ArmController arm_controller = ArmController();
  ros::Subscriber PosCommandSub_ = arm_controller.nh_.subscribe("robotino_arm/joint_control/move", 1, &ArmController::PosCommandSub_cb, &arm_controller);
  ros::Publisher cmdVelPub_ = arm_controller.nh_.advertise<geometry_msgs::Twist>("cmd_rotatory", 1, &arm_controller);
  ros::Publisher ArmPosCommandPub_ = arm_controller.nh_.advertise<std_msgs::Float64MultiArray>("arm_controller/group_position_controller/cmd", 1, &arm_controller);
  ros::spin();
  return 0;
}
