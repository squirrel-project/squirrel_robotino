/*
 * RobotinoSafety.h
 *
 *  Created on: Mar 21, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOSAFETY_H_
#define ROBOTINOSAFETY_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>

class RobotinoSafety
{
 public:
  
	RobotinoSafety( void );
  
	~RobotinoSafety( void );

	void spin();

 private:
  void calcScale( void );

  void buildEllipseVizMsgs( void );

	void cmdVelCallback( const geometry_msgs::TwistConstPtr& );

  void bumperCallback( const std_msgs::BoolConstPtr& );

  void scanCallback( const sensor_msgs::LaserScanConstPtr& );

	void check( sensor_msgs::PointCloud );

	void inE2( geometry_msgs::Point32 );

  double solveE1( geometry_msgs::Point32 );

	void visualizeEllipses(bool show = true );

	ros::NodeHandle nh_;

	ros::Publisher cmd_vel_pub_;
	ros::Publisher e1_viz_pub_;
	ros::Publisher e2_viz_pub_;

	ros::Subscriber move_base_cmd_vel_sub_;
	ros::Subscriber bumper_sub_;
	ros::Subscriber scan_sub_;

	geometry_msgs::Twist controller_vel_msg_;

	visualization_msgs::Marker e1_viz_msg_, e2_viz_msg_;

	laser_geometry::LaserProjection projector_;
	tf::TransformListener tfListener_;
  
	bool stop_laser_, slow_laser_, bumper_;

	double scale_, dist_;

	// params
	double e1_major_radius_, e1_minor_radius_;
	double e2_major_radius_, e2_minor_radius_;
  double node_loop_rate_;
  bool use_safe_vel_;
  std::string controller_vel_topic_, bumper_topic_, scan_topic_;
};

#endif /* ROBOTINOSAFETY_H_ */
