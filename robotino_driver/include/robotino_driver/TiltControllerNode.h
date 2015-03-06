// TiltControllerNode.h --- 
// 
// Filename: TiltControllerNode.h
// Description: Reset the tilt controller to starting angle
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Mar  3 11:58:38 2015 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Mar 4 09:33:52 2015 (+0100)
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

#ifndef TILTSTARTUPNODE_H_
#define TILTSTARTUPNODE_H_

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>

#include <std_srvs/Empty.h>

#define PI 3.14159265358979

class TiltControllerNode
{
 public:
  TiltControllerNode( void );

  virtual ~TiltControllerNode( void );

  void spin( void );

  double startupAngle( void );
  
 private:
  bool resetKinectPosition( std_srvs::Empty::Request&, std_srvs::Empty::Response& );
  
  ros::NodeHandle private_nh_, public_nh_;
  ros::Publisher tilt_pub_;
  ros::ServiceServer reset_tilt_srv_;
  
  std_msgs::Float64 desired_angle_msg_;
  
  std::string command_topic_, status_topic_;
};

#endif /* TILTSTARTUPNODE_H_ */

// 
// TiltControllerNode.h ends here
