/**
 * ViewControllerNode.h
 *
 * Sets the view of the robot's camera. This will typically use the pan and
 * tilt joints, but could actually also use the robot base if needed.
 * It will take view requests from clients and decide, also based on THEORY
 * priority of requests, where to move the camera.
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <cmath>
#include <string>
#include "ViewControllerNode.h"

ViewControllerNode::ViewControllerNode() :
    private_nh_("~")
{
  pan_ = tilt_ = 0.;

  private_nh_.param<float>("default_pan_angle", default_pan_, 0.0);
  private_nh_.param<float>("default_tilt_angle", default_tilt_, 0.0);
  private_nh_.param<std::string>("pan_command_topic", pan_command_topic_, "/pan_controller/command");
  private_nh_.param<std::string>("pan_status_topic", pan_status_topic_, "/pan_controller/state");
  private_nh_.param<std::string>("tilt_command_topic", tilt_command_topic_, "/tilt_controller/command");
  private_nh_.param<std::string>("tilt_status_topic", tilt_status_topic_, "/tilt_controller/state");

  pan_pub_ = public_nh_.advertise<std_msgs::Float64>(pan_command_topic_, 2);
  tilt_pub_ = public_nh_.advertise<std_msgs::Float64>(tilt_command_topic_, 2);
  pan_state_sub_ = public_nh_.subscribe(pan_status_topic_, 2, &ViewControllerNode::panStateCallback, this);
  tilt_state_sub_ = public_nh_.subscribe(tilt_status_topic_, 2, &ViewControllerNode::tiltStateCallback, this);

  look_image_srv_ = public_nh_.advertiseService("/view_controller/look_at_image_position", &ViewControllerNode::lookAtImagePosition, this);
  look_srv_ = public_nh_.advertiseService("/view_controller/look_at_position", &ViewControllerNode::lookAtPosition, this);
  fixate_srv_ =  public_nh_.advertiseService("/view_controller/fixate_position", &ViewControllerNode::fixatePosition, this);
  fixate_pantilt_srv_ =  public_nh_.advertiseService("/view_controller/fixate_pantilt", &ViewControllerNode::fixatePanTilt, this);
  clear_srv_ = public_nh_.advertiseService("/view_controller/clear_fixation", &ViewControllerNode::clearFixation, this);
  reset_srv_ = public_nh_.advertiseService("/view_controller/reset", &ViewControllerNode::resetPosition, this);
}

ViewControllerNode::~ViewControllerNode()
{
  public_nh_.shutdown();
  private_nh_.shutdown();
}

void ViewControllerNode::init()
{
  bool have_all = true;
  if ( !ros::topic::waitForMessage<dynamixel_msgs::JointState>(pan_status_topic_, ros::Duration(30.0)) ) {
    ROS_WARN("pan controller not running, shutting down the node");
    have_all = false;
  }
  if ( !ros::topic::waitForMessage<dynamixel_msgs::JointState>(tilt_status_topic_, ros::Duration(30.0)) ) {
    ROS_WARN("tilt controller not running, shutting down the node");
    have_all = false;
  }
  if(have_all) {
    joint_mutex_.lock();
    ROS_INFO("moving to default pan/tilt position: %f / %f [rad])",
      default_pan_, default_tilt_);
    movePanTilt(default_pan_, default_tilt_);
    joint_mutex_.unlock();
  }
  else {  
    ros::shutdown();
  }
}

void ViewControllerNode::panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg)
{
  joint_mutex_.lock();
  pan_ = panStateMsg->current_pos;
  joint_mutex_.unlock();
}

void ViewControllerNode::tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg)
{
  joint_mutex_.lock();
  tilt_ = tiltStateMsg->current_pos;
  joint_mutex_.unlock();
}

bool ViewControllerNode::resetPosition( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  joint_mutex_.lock();
  ROS_INFO("moving to default pan/tilt position: %f / %f [rad])",
    default_pan_, default_tilt_);
  movePanTilt(default_pan_, default_tilt_);
  who_fixed_it = "";
  joint_mutex_.unlock();
  return true;
}

bool ViewControllerNode::lookAtImagePosition(squirrel_object_perception_msgs::LookAtImagePosition::Request &req,
                                             squirrel_object_perception_msgs::LookAtImagePosition::Response &res)
{
  if(who_fixed_it.empty())
  {
    joint_mutex_.lock();
    // HACK: the focal length is hardcoded for the Kinect/Asus
    movePanTilt(pan_ - atan2(req.x, 525), tilt_ + atan2(req.y, 525));
    //ROS_INFO("pan/tilt relative move move (deg): %.f %.f", -atan2(req.x, 525)*180./M_PI, atan2(req.y, 525*180./M_PI));
    joint_mutex_.unlock();
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewControllerNode::lookAtPosition(squirrel_object_perception_msgs::LookAtPosition::Request &req,
                                        squirrel_object_perception_msgs::LookAtPosition::Response &res)
{
  // TODO
  return false;
}

bool ViewControllerNode::fixatePosition(squirrel_object_perception_msgs::FixatePosition::Request &req,
                                        squirrel_object_perception_msgs::FixatePosition::Response &res)
{
  // TODO
  return false;
}

bool ViewControllerNode::fixatePanTilt(squirrel_object_perception_msgs::FixatePanTilt::Request& req,
				       squirrel_object_perception_msgs::FixatePanTilt::Response& res)
{
  if(who_fixed_it.empty())
  {
    joint_mutex_.lock();
    movePanTilt(req.pan, req.tilt);
    joint_mutex_.unlock();
    if(!req.why.empty())
      who_fixed_it = req.why;
    else
      who_fixed_it = "*";
    return true;
  }
  else
  {
    ROS_INFO("view is already fixed by '%s'", who_fixed_it.c_str());
    return false;
  }
}

bool ViewControllerNode::clearFixation(squirrel_object_perception_msgs::ClearFixation::Request &req,
                                       squirrel_object_perception_msgs::ClearFixation::Response &res)
{
  if(!who_fixed_it.empty())
  {
    std::string why = req.why;
    if(why.empty())
      why = "*";
    if(who_fixed_it == req.why)
    {
      who_fixed_it = "";
      return true;
    }
    else
    {
      ROS_INFO("view was fixed by '%s', but you are '%s', ignoring", who_fixed_it.c_str(), req.why.c_str());
      return false;
    }
  }
  else
  {
    ROS_INFO("view was not fixed, ignoring");
    return false;
  }
}

void ViewControllerNode::movePanTilt(float pan, float tilt)
{
  std_msgs::Float64 panMsg, tiltMsg;
  panMsg.data = pan;
  tiltMsg.data = tilt;
  if(std::isfinite(panMsg.data) && std::isfinite(tiltMsg.data))
  {
    pan_pub_.publish(panMsg);
    tilt_pub_.publish(tiltMsg);
  }
}
