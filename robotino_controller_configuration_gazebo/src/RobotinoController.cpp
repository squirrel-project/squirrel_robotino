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
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.14159265358979

namespace robotino_controller_configuration_gazebo {

RobotinoController::RobotinoController(ros::NodeHandle& nh) :
	nh_(nh),
	x_(0.f), y_(0.f), phi_(0.f),
	vx_(0.f), vy_(0.f), omega_(0.f),
	current_time_(ros::Time::now()), last_time_(ros::Time::now()), 
    gear_(16.0),	//16
    rw_(0.04),  //40),
    rb_(0.132) //132)
{
	odometry_publisher_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
	command_w_.data.assign(3,0);
	cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RobotinoController::setVelocity, this);
	w_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 10);
}

RobotinoController::~RobotinoController( void )
{
	nh_.shutdown();
}

void RobotinoController::spin( void )
{
	ros::Rate lr(10);
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();


	while ( ros::ok() )
	{

	command_w_.data[0] = 0.0;
	command_w_.data[1] = 0.0;
	command_w_.data[2] = 0.0;

	w_pub_.publish(command_w_);
   
   //UpdateOdometry();

   ros::spinOnce();
   lr.sleep();
	}  
}

void RobotinoController::setVelocity( const geometry_msgs::TwistConstPtr& cmd_vel_msg )
{

	boost::mutex::scoped_lock lock(velocity_mutex_);
	vx_ = cmd_vel_msg->linear.x;
	vy_ = cmd_vel_msg->linear.y;
	omega_ = cmd_vel_msg->angular.z; 

	// ROS_INFO("input velocities: (%f, %f, %f)", vx_, vy_, omega_);
 
	double v0[2] = { -0.5 * sqrt( 3.0 ),  0.5 };
	double v1[2] = {  0.0              , -0.5 * sqrt( 3.0 ) };
	double v2[2] = {  0.5 * sqrt( 3.0 ),  0.5 };
	
	// scale omega with the radius of the robot
	double v_omega_scaled = rb_ * (double)omega_ ;
	double k = 1 / rw_ ;
	
	command_w_.data[0] = ( v0[0] * (double)vx_ + v0[1] * (double)vy_ + v_omega_scaled ) * k;
	command_w_.data[1] = ( v1[0] * (double)vx_ + v1[1] * (double)vy_ + v_omega_scaled ) * k;
	command_w_.data[2] = ( v2[0] * (double)vx_ + v2[1] * (double)vy_ + v_omega_scaled ) * k;

	w_pub_.publish(command_w_);

}

void RobotinoController::UpdateOdometry()
{
	boost::mutex::scoped_lock lock(velocity_mutex_);

	//compute odometry in a typical way given the velocities of the robot
	current_time_ = ros::Time::now();
	double dt = (current_time_ - last_time_).toSec();
	double delta_x = (vx_ * cos(phi_) - vy_ * sin(phi_)) * dt;
	double delta_y = (vx_ * sin(phi_) + vy_ * cos(phi_)) * dt;
	double delta_phi = omega_ * dt;

	x_ += delta_x;
	y_ += delta_y;
	phi_ += delta_phi;
	
	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi_);
	
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x_;
	odom_trans.transform.translation.y = y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	tf_broadcaster_.sendTransform(odom_trans);
	
	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time_;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x_;
	odom.pose.pose.position.y = y_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx_;
	odom.twist.twist.linear.y = vy_;
	odom.twist.twist.angular.z = omega_;

	//publish the message
	odometry_publisher_.publish(odom);
	
	last_time_ = current_time_;
}


}	// namespace robotino_controller_configuration_gazebo //

// RobotinoController.cpp ends here
