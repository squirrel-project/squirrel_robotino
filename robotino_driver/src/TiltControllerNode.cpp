// TiltControllerNode.cpp --- 
// 
// Filename: TiltControllerNode.cpp
// Description: Reset the tilt controller to starting angle
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Mar  3 12:01:44 2015 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Mar 4 09:32:00 2015 (+0100)
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility:
//   tested on
//    - ROS Hydro 
//    - ROS Indigo
// 

// Copyright (c) 2015, Federico Boniardi
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

#include "TiltControllerNode.h"

#include <string>

TiltControllerNode::TiltControllerNode( void ) :
    private_nh_("~")
{
  private_nh_.param("startup_angle", desired_angle_msg_.data, PI/4.0);
  private_nh_.param<std::string>("tilt_command_topic", command_topic_, "/tilt_controller/command");
  private_nh_.param<std::string>("tilt_status_topic", status_topic_, "/tilt_controller/state");
 
  tilt_pub_ = public_nh_.advertise<std_msgs::Float64>("/tilt_controller/command", 2);
  reset_tilt_srv_ = public_nh_.advertiseService("/tilt_controller/resetPosition", &TiltControllerNode::resetKinectPosition, this);
  
  ros::spinOnce();
}

TiltControllerNode::~TiltControllerNode( void )
{
  public_nh_.shutdown();
  private_nh_.shutdown();
}

void TiltControllerNode::spin( void )
{
  if ( !ros::topic::waitForMessage<dynamixel_msgs::JointState>(status_topic_, ros::Duration(30.0)) ) {
    ROS_WARN("tilt controller not running, shutting down the node");
    ros::shutdown();
  } else {
    tilt_pub_.publish(desired_angle_msg_);
    ROS_INFO("moving Kinect to starting position (%f rad)", desired_angle_msg_.data);
  }

  ros::Rate lr(1.0);
  while ( private_nh_.ok() ) {     
    ros::spinOnce();
    lr.sleep();
  }
}

double TiltControllerNode::startupAngle( void )
{
  return desired_angle_msg_.data;
}

bool TiltControllerNode::resetKinectPosition( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  tilt_pub_.publish(desired_angle_msg_);
  ROS_INFO("moving Kinect to starting position (%f rad)", desired_angle_msg_.data);
  return true;
}

// 
// TiltControllerNode.cpp ends here
