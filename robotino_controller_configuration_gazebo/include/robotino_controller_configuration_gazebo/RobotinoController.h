// RobotinoController.h --- 
// 
// Filename: RobotinoController.h
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

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#ifndef ROBOTINOCONTROLLER_H_
#define ROBOTINOCONTROLLER_H_

namespace robotino_controller_configuration_gazebo {

class RobotinoController
{
 public:
  RobotinoController( void );
  virtual ~RobotinoController( void );
  void spin( void );
 
private:
  void setVelocity( const geometry_msgs::TwistConstPtr& );
  
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher w0_pub_, w1_pub_, w2_pub_;
  float rw_, rb_, gear_, k_;
  std_msgs::Float64 w0_, w1_, w2_;
};

} // namespace robotino_controller_configuration_gazebo

#endif /* ROBOTINOCONTROLLER_H_ */

// 
// RobotinoController.h ends here
