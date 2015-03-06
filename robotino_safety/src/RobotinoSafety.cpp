/*
 * RobotinoSafety.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: indorewala@servicerobotics.euo
 */

#include "RobotinoSafety.h"

#include <string>

#define PI 3.141592653

// e1 is the inner ellipse
// e2 is the outer ellipse

RobotinoSafety::RobotinoSafety( void ):
	nh_("~"),
	stop_laser_(false),
	slow_laser_(false),
	e1_major_radius_(0.40),
	e1_minor_radius_(0.25),
	e2_major_radius_(0.70),
	e2_minor_radius_(0.30),
	node_loop_rate_ (20.0),
  controller_vel_topic_("/robotino_cmd_vel"),
  bumper_topic_("/bumper"),
  scan_topic_("/scan"),
  use_safe_vel_(false),
  bumper_(false)
{
  nh_.param<std::string>("controller_vel_topic", controller_vel_topic_, "/robotino_cmd_vel");
  nh_.param<std::string>("bumper_topic", bumper_topic_, "/bumper");
  nh_.param<std::string>("scan_topic", scan_topic_, "/scan");

  nh_.param<bool>("use_safe_velocity", use_safe_vel_, false);
  
  nh_.param<double>("outer_major_radius", e2_major_radius_, 0.70);
	nh_.param<double>("outer_minor_radius", e2_minor_radius_, 0.30);
	nh_.param<double>("inner_major_radius", e1_major_radius_, 0.40);
	nh_.param<double>("inner_minor_radius", e1_minor_radius_, 0.25);
	nh_.param<double>("node_loop_rate", node_loop_rate_, 20.0);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(controller_vel_topic_, 1);
	e1_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("inner_ellipse_marker", 10);
	e2_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("outer_ellipse_marker", 10);
      
  move_base_cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &RobotinoSafety::cmdVelCallback, this);
	bumper_sub_ = nh_.subscribe(bumper_topic_, 1, &RobotinoSafety::bumperCallback, this);
	scan_sub_ = nh_.subscribe(scan_topic_, 1, &RobotinoSafety::scanCallback, this);
 
	calcScale();
	buildEllipseVizMsgs();
}

RobotinoSafety::~RobotinoSafety( void )
{
	cmd_vel_pub_.shutdown();
	move_base_cmd_vel_sub_.shutdown();
	bumper_sub_.shutdown();
	scan_sub_.shutdown();
}

void RobotinoSafety::spin( void )
{
	ros::Rate lr(node_loop_rate_);
	while( nh_.ok() ) {
		ros::spinOnce();
		lr.sleep();
	}
}

void RobotinoSafety::calcScale( void )
{
	geometry_msgs::Point32 point_on_e2;

	point_on_e2.x = e2_major_radius_;
	point_on_e2.y = 0;

	scale_ = solveE1(point_on_e2);
	dist_ = scale_;
}

void RobotinoSafety::buildEllipseVizMsgs( void ) 
{
	e1_viz_msg_.header.frame_id = e2_viz_msg_.header.frame_id = "/base_link";
	e1_viz_msg_.header.stamp = e2_viz_msg_.header.stamp = ros::Time::now();
	e1_viz_msg_.ns = "inner_ellipse";
	e2_viz_msg_.ns = "outer_ellipse";

	e1_viz_msg_.action = e2_viz_msg_.action = visualization_msgs::Marker::ADD;
	e1_viz_msg_.type = e2_viz_msg_.type = visualization_msgs::Marker::POINTS;

	// Color the ellipses green
	e1_viz_msg_.color.g = 1.0;
	e1_viz_msg_.color.a = 1.0;
	e2_viz_msg_.color.g = 1.0;
	e2_viz_msg_.color.a = 1.0;

	// Scale the points
	e1_viz_msg_.scale.x = 0.02;
	e1_viz_msg_.scale.y = 0.02;

	e2_viz_msg_.scale.x = 0.02;
	e2_viz_msg_.scale.y = 0.02;

	// Now we populate the msgs
	for( double t = -PI/2; t <= PI/2; t += 0.1 )
	{
		geometry_msgs::Point e1_p, e2_p;

		e1_p.x = e1_major_radius_ * cos(t);
		e1_p.y = e1_minor_radius_ * sin(t);

		e2_p.x = e2_major_radius_ * cos(t);
		e2_p.y = e2_minor_radius_ * sin(t);

		e1_viz_msg_.points.push_back(e1_p);
		e2_viz_msg_.points.push_back(e2_p);
	}
}

void RobotinoSafety::cmdVelCallback( const geometry_msgs::TwistConstPtr& cmd_vel_msg )
{
  if ( bumper_ ) {
		ROS_WARN("Bumper hit! Stopping the robot!");
    controller_vel_msg_.linear.x = 0.0;
    controller_vel_msg_.linear.y = 0.0;
    controller_vel_msg_.angular.z = 0.0;
  } else if ( use_safe_vel_ ) {
    controller_vel_msg_.linear.x = ( dist_ / scale_ ) * cmd_vel_msg->linear.x;
    controller_vel_msg_.linear.y = ( dist_ / scale_ ) * cmd_vel_msg->linear.y;
    controller_vel_msg_.angular.z = ( dist_ / scale_ ) * cmd_vel_msg->angular.z;
  } else {
    controller_vel_msg_.linear.x = cmd_vel_msg->linear.x;
    controller_vel_msg_.linear.y = cmd_vel_msg->linear.y;
    controller_vel_msg_.angular.z = cmd_vel_msg->angular.z;
  }
  
  cmd_vel_pub_.publish(controller_vel_msg_);
}

void RobotinoSafety::bumperCallback( const std_msgs::BoolConstPtr& msg )
{
	if( msg->data ) {
    bumper_ = true;
  } else {
    bumper_ = false;
  }
}

void RobotinoSafety::scanCallback( const sensor_msgs::LaserScanConstPtr& msg )
{
	sensor_msgs::PointCloud cloud;

	try {
		tfListener_.waitForTransform("/base_link", "/hokuyo_link", ros::Time(0), ros::Duration(1.0));
		projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, tfListener_);
	} catch(tf::LookupException& ex) {
		ROS_WARN("Lookup exception: %s", ex.what());
		return;
	} catch(tf::ConnectivityException& ex) {
		ROS_WARN("Connectivity exception: %s", ex.what());
		return;
	} catch(tf::ExtrapolationException& ex) {
		ROS_WARN("Extrapolation exception: %s", ex.what());
		return;
	}
  
	check(cloud);
	visualizeEllipses();
}

void RobotinoSafety::check( sensor_msgs::PointCloud cloud )
{
	stop_laser_ = false;
	slow_laser_ = false;
	dist_ = scale_;

	for(unsigned int i=0; i < cloud.points.size(); ++i)
	{
		inE2(cloud.points[i]);
	}
}

void RobotinoSafety::inE2( geometry_msgs::Point32 point )
{
	double check = pow( (point.x / e2_major_radius_), 2 ) + pow( (point.y / e2_minor_radius_), 2 ) - 1;

	if ( check <= 0.0 )
  {
		slow_laser_ = true;
    
		dist_ = solveE1(point);

    if(dist_ <= 0.0 )
    {
			dist_ = 0.0;
			stop_laser_ = true;
		}
		return;
	}
}

double RobotinoSafety::solveE1( geometry_msgs::Point32 point )
{
	return ( pow( (point.x / e1_major_radius_), 2 ) + pow( (point.y / e1_minor_radius_), 2 ) - 1 );
}

void RobotinoSafety::visualizeEllipses( bool show )
{
	e1_viz_msg_.header.stamp = e2_viz_msg_.header.stamp = ros::Time::now();

	// Color the ellipses green
	e1_viz_msg_.color.r = 0.0;
	e1_viz_msg_.color.g = 1.0;
	e1_viz_msg_.color.b = 0.0;

	e2_viz_msg_.color.r = 0.0;
	e2_viz_msg_.color.g = 1.0;
	e2_viz_msg_.color.b = 0.0;

	if(stop_laser_)
	{
		// Color the ellipse e1 red
		e1_viz_msg_.color.r = 1.0;
		e1_viz_msg_.color.g = 0.0;
		e1_viz_msg_.color.b = 0.0;
	}
	if (slow_laser_)
	{
		// Color the ellipse e2 red
		e2_viz_msg_.color.r = 1.0;
		e2_viz_msg_.color.g = 0.0;
		e2_viz_msg_.color.b = 0.0;
	}

	e1_viz_pub_.publish(e1_viz_msg_);
	e2_viz_pub_.publish(e2_viz_msg_);
}
