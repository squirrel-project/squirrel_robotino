// RobotinoController.cpp --- 
// 
// Filename: RobotinoController.cpp
// Description: Porting of Robotino's driver for Gazebo
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Dec 16 10:06:54 2014 (+0100)
// Version: 
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Copyright (c) 2014, Federico Boniardi
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 

// Code:

#include "robotino_controller_configuration_gazebo/RobotinoController.h"

#include <ros/ros.h>

#define PI 3.14159265358979

namespace robotino_controller_configuration_gazebo {

RobotinoController::RobotinoController( void ) :
    gear_(16.0),
    rw_(0.040),
    rb_(0.132)
{
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1000, &RobotinoController::setVelocity, this);
  w0_pub_ = nh_.advertise<std_msgs::Float64>("/wheel0", 1000);
  w1_pub_ = nh_.advertise<std_msgs::Float64>("/wheel1", 1000);
  w2_pub_ = nh_.advertise<std_msgs::Float64>("/wheel2", 1000);
}

RobotinoController::~RobotinoController( void )
{
  nh_.shutdown();
}

void RobotinoController::spin( void )
{
  ros::Rate lr(10);
  while ( ros::ok() ) {     
    ros::spinOnce();
    lr.sleep();
  }  
}

void RobotinoController::setVelocity( const geometry_msgs::TwistConstPtr& cmd_vel_msg )
{
  float vx = cmd_vel_msg->linear.x;
  float vy = cmd_vel_msg->linear.y;
  float omega = cmd_vel_msg->angular.z; 

  // ROS_INFO("input velocities: (%f, %f, %f)", vx, vy, omega);
 
  double v0[2] = { -0.5 * sqrt( 3.0 ),  0.5 };
  double v1[2] = {  0.0              , -1.0 };
  double v2[2] = {  0.5 * sqrt( 3.0 ),  0.5 };

  // ROS_INFO("v0[0] = %f", v0[0]);
  // ROS_INFO("v0[1] = %f", v0[1]);
  // ROS_INFO("v1[0] = %f", v1[0]);
  // ROS_INFO("v1[1] = %f", v0[1]);
  // ROS_INFO("v2[0] = %f", v2[0]);
  // ROS_INFO("v2[1] = %f", v2[1]);
  
  double v_omega_scaled = rb_ * (double)omega ;
  
  w0_.data = ( v0[0] * (double)vx + v0[1] * (double)vy + v_omega_scaled ) * gear_;
  w1_.data = ( v1[0] * (double)vx + v1[1] * (double)vy + v_omega_scaled ) * gear_;
  w2_.data = ( v2[0] * (double)vx + v2[1] * (double)vy + v_omega_scaled ) * gear_;

  // ROS_INFO("output velocities: (%f, %f, %f)", w0_.data, w1_.data, w2_.data);
  
  w0_pub_.publish(w0_);
  w1_pub_.publish(w1_);
  w2_pub_.publish(w2_);
}

}  // namespace robotino_controller_configuration_gazebo //

// RobotinoController.cpp ends here
