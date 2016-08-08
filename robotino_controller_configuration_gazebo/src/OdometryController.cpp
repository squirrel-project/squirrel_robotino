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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#define PI 3.14159265
#define gear 16
#define rb 0.132
#define rw 0.04

using namespace std;
class odometry_publisher
{
  double theta_rob_rad_;
  double y_rear;
  double vy_rear;
  nav_msgs::Odometry odom_;
  public:
   	ros::NodeHandle n;
   	ros::Publisher odom_pub;
   	tf::TransformBroadcaster odom_broadcaster;
   	void joint_stateCallback(const sensor_msgs::JointState msg);
};

void odometry_publisher::joint_stateCallback(const sensor_msgs::JointState msg)
{
  double pos_wheel0 = msg.position[0];
  double pos_wheel1 = msg.position[1];
  double pos_wheel2 = msg.position[2];

  double vel_wheel0 = msg.velocity[0];
  double vel_wheel1 = msg.velocity[1];  
  double vel_wheel2 = msg.velocity[2];

  double dt= 0.01;

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1/dt);
  //double now = ros::Time::now():

  odom_.header.stamp = ros::Time::now();
  odom_.header.frame_id = "/odom";
  odom_.child_frame_id = "/base_link";

   //Convert from rad/s to m/s
  double k = 1 / rw ;
  double vx_rear = static_cast<float>( ( (double)vel_wheel2 - (double)vel_wheel0 ) / sqrt( 3.0 ) / k );
  double vy_rear = static_cast<float>( 2.0 / 3.0 * ( (double)vel_wheel0 + 0.5 * ( (double)vel_wheel2 - (double)vel_wheel0 ) - (double)vel_wheel1 ) / k );
  double vw = vy_rear + (double)vel_wheel1 / k; 
  double theta_vel = static_cast<float>( vw / rb );

  //double theta_vel = -(tan(pos_wheel_yaw) * vel_wheel_x*r1)/(L1 + tan(pos_wheel_yaw)*sin(pos_wheel_yaw)*sqrt(pow(L1,2)+pow(l1,2)));  
  //double vx_rear = -(theta_vel * L1) / tan(pos_wheel_yaw);
  double vx_rear_mid = (vx_rear+odom_.twist.twist.linear.x)/2.0;
  double vy_rear_mid = (vy_rear+odom_.twist.twist.linear.y)/2.0;
  odom_.twist.twist.linear.x = vx_rear_mid;
  odom_.twist.twist.linear.y = vy_rear_mid;
  odom_.twist.twist.angular.z = theta_vel;
  
  double sin_theta = sin(theta_rob_rad_);
  double cos_theta = cos(theta_rob_rad_);
  theta_rob_rad_ += theta_vel * dt;

  double dx = (vx_rear_mid * cos_theta - vy_rear_mid * sin_theta) * dt;
  //cout << "dx " << dx << "\n" << " ";
  double dy = (vx_rear_mid * sin_theta + vy_rear_mid * cos_theta) * dt;
  //cout << "dy " << dy << "\n" << " ";

  odom_.pose.pose.position.x += dx;
  odom_.pose.pose.position.y += dy;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_rob_rad_);

  odom_pub.publish(odom_);

  geometry_msgs::TransformStamped odom_tf_;
  // compose and publish transform for tf package
  // compose header
  odom_tf_.header.stamp = odom_.header.stamp;
  odom_tf_.header.frame_id = "/odom";
  odom_tf_.child_frame_id = "/base_link";
  // compose data container
  odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf_.transform.rotation = odom_.pose.pose.orientation;

  // publish the transform (for debugging, conflicts with robot-pose-ekf)
  odom_broadcaster.sendTransform(odom_tf_);
  ros::spinOnce();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  odometry_publisher odomPub = odometry_publisher();
  ros::Subscriber sub = odomPub.n.subscribe("joint_states", 100, &odometry_publisher::joint_stateCallback, &odomPub);
  ros::spin();

  return 0;

}

