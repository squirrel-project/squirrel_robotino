/****************************************************************
 *
 * Copyright (c) 2015
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: squirrel
 * ROS stack name: squirrel_robotino
 * ROS package name: robotino_calibration
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
 *
 * Date of creation: December 2015
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

#include <robotino_calibration/camera_base_calibration.h>

#include <std_msgs/Float64.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <sstream>


cv::Mat rotationMatrixFromRPY(double roll, double pitch, double yaw)
{
	double sr = sin(roll);
	double cr = cos(roll);
	double sp = sin(pitch);
	double cp = cos(pitch);
	double sy = sin(yaw);
	double cy = cos(yaw);
	cv::Mat rotation = (cv::Mat_<double>(3,3) <<
			cr*cp,		cr*sp*sy - sr*cy,		cr*sp*cy + sr*sy,
			sr*cp,		sr*sp*sy + cr*cy,		sr*sp*cy - cr*sy,
			-sp,		cp*sy,					cp*cy);

	return rotation;
}

cv::Mat makeTransform(const cv::Mat& R, const cv::Mat& t)
{
	cv::Mat T = (cv::Mat_<double>(4,4) <<
			R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
			R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
			R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2),
			0., 0., 0., 1);
	return T;
}

CameraBaseCalibration::CameraBaseCalibration(ros::NodeHandle nh) :
			node_handle_(nh), transform_listener_(nh), camera_calibration_path_("robotino_calibration/camera_calibration/"),
			tilt_controller_command_("/tilt_controller/command"), pan_controller_command_("/pan_controller/command"), capture_image_(true)
{
//	// load parameters
//	node_handle_.param("chessboard_cell_size", chessboard_cell_size_, 0.05);
//	std::vector<double> temp;
//	node_handle_.param("chessboard_pattern_size", temp);
//	chessboard_pattern_size_ = cv::Size(temp[0], temp[1]);
//	std::cout << "pattern: " << chessboard_pattern_size_ << std::endl;
//
//	getchar();

	chessboard_cell_size_ = 0.05;
	chessboard_pattern_size_ = cv::Size(6,4);

	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "colorimage_in", 1);
	color_image_sub_.registerCallback(boost::bind(&CameraBaseCalibration::imageCallback, this, _1));

	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);
	base_controller_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);

	// todo: parameters
	torso_lower_frame_ = "base_pan_link";
	torso_upper_frame_ = "tilt_link";
	camera_frame_ = "kinect_link";
	camera_optical_frame_ = "kinect_rgb_optical_frame";
	base_frame_ = "base_link";
	checkerboard_frame_ = "checkerboard";

	// todo: set good initial parameters
	T_base_to_torso_lower_ = makeTransform(rotationMatrixFromRPY(0.0, 0.0, 0.0), cv::Mat(cv::Vec3d(0.25, 0, 0.5)));
	T_torso_upper_to_camera_ = makeTransform(rotationMatrixFromRPY(0.0, 0.0, -1.57), cv::Mat(cv::Vec3d(0.0, 0.065, 0.0)));
}

CameraBaseCalibration::~CameraBaseCalibration()
{
	if (it_ != 0)
		delete it_;
}

bool CameraBaseCalibration::convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);//image_msg->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ImageFlip::convertColorImageMessageToMat: cv_bridge exception: %s", e.what());
		return false;
	}
	image = image_ptr->image;

	return true;
}

void CameraBaseCalibration::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	// secure this access with a mutex
	boost::mutex::scoped_lock lock(camera_data_mutex_);

	if (capture_image_ == true)
	{
		// read image
		cv_bridge::CvImageConstPtr color_image_ptr;
		if (convertImageMessageToMat(color_image_msg, color_image_ptr, camera_image_) == false)
			return;

		latest_image_time_ = color_image_msg->header.stamp;

		capture_image_ = false;
	}
}

bool CameraBaseCalibration::calibrateCameraToBase(const bool load_images)
{
	// setup storage folder
	int return_value = system("mkdir -p robotino_calibration/camera_calibration");

	// pre-cache images
	if (load_images == false)
	{
		ros::spinOnce();
		ros::Duration(2).sleep();
		capture_image_ = true;
		ros::spinOnce();
		ros::Duration(2).sleep();
		capture_image_ = true;
	}

	// acquire images
	int image_width=0, image_height=0;
	std::vector< std::vector<cv::Point2f> > points_2d_per_image;
	std::vector<cv::Mat> T_base_to_checkerboard_vector;
	std::vector<cv::Mat> T_torso_lower_to_torso_upper_vector;
	std::vector<cv::Mat> T_camera_to_camera_optical_vector;
	acquireCalibrationImages(image_width, image_height, points_2d_per_image, T_base_to_checkerboard_vector,
			T_torso_lower_to_torso_upper_vector, T_camera_to_camera_optical_vector, chessboard_pattern_size_, load_images);

	// prepare chessboard 3d points
	std::vector< std::vector<cv::Point3f> > pattern_points_3d;
	computeCheckerboard3dPoints(pattern_points_3d, chessboard_pattern_size_, chessboard_cell_size_, points_2d_per_image.size());

	// intrinsic calibration for camera
	std::vector<cv::Mat> rvecs, tvecs, T_camera_to_checkerboard_vector;
	intrinsicCalibration(pattern_points_3d, points_2d_per_image, cv::Size(image_width, image_height), rvecs, tvecs);
	for (size_t i=0; i<rvecs.size(); ++i)
	{
		cv::Mat R, t;
		cv::Rodrigues(rvecs[i], R);
		cv::Mat T_camera_to_checkerboard = T_camera_to_camera_optical_vector[i] * makeTransform(R, tvecs[i]);
		T_camera_to_checkerboard_vector.push_back(T_camera_to_checkerboard);
	}

	// extrinsic calibration between base and torso_lower as well ass torso_upper and camera
	for (int i=0; i<5; ++i)
	{
		std::cout << "\nExtrinsic optimization run " << i << ":" << std::endl;
		extrinsicCalibrationBaseToTorsoLower(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
		std::cout << "T_base_to_torso_lower:\n" << T_base_to_torso_lower_ << std::endl;
		// todo: make a function
		Eigen::Matrix3f rot;
		for (int i=0; i<3; ++i)
			for (int j=0; j<3; ++j)
				rot(i,j) = T_base_to_torso_lower_.at<double>(i,j);
		Eigen::Vector3f euler_angles = rot.eulerAngles(2,1,0);
		std::cout << "yaw=" << euler_angles(0) << "   pitch=" << euler_angles(1) << "   roll=" << euler_angles(2) << std::endl;
		extrinsicCalibrationTorsoUpperToCamera(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
		std::cout << "T_torso_upper_to_camera:\n" << T_torso_upper_to_camera_ << std::endl;
		for (int i=0; i<3; ++i)
			for (int j=0; j<3; ++j)
				rot(i,j) = T_torso_upper_to_camera_.at<double>(i,j);
		euler_angles = rot.eulerAngles(2,1,0);
		std::cout << "yaw=" << euler_angles(0) << "   pitch=" << euler_angles(1) << "   roll=" << euler_angles(2) << std::endl;
	}


//	// extrinsic calibration from fz to jai
//	extrinsicCalibration(fz_points_3d_vector, points_2d_vector);
//
//	// camera-robot calibration
//	cameraRobotCalibration(rvecs, tvecs);

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return true;
}

bool CameraBaseCalibration::getTransform(const std::string& target_frame, const std::string& source_frame, cv::Mat& T)
{
	try
	{
		tf::StampedTransform Ts;
		transform_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
		transform_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
		const tf::Matrix3x3& rot = Ts.getBasis();
		const tf::Vector3& trans = Ts.getOrigin();
		cv::Mat rotcv(3,3,CV_64FC1);
		cv::Mat transcv(3,1,CV_64FC1);
		for (int v=0; v<3; ++v)
			for (int u=0; u<3; ++u)
				rotcv.at<double>(v,u) = rot[v].m_floats[u];
		for (int v=0; v<3; ++v)
			transcv.at<double>(v) = trans.m_floats[v];
		T = makeTransform(rotcv, transcv);
		//std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}

	return true;
}

bool CameraBaseCalibration::moveRobot(const RobotConfiguration& robot_configuration)
{
	// do not move if close to goal
	double error_phi = 10;
	double error_x = 10;
	double error_y = 10;
	cv::Mat T;
	if (!getTransform("checkerboard_reference_nav", "base_link", T))
		return false;
	// todo: make a conversion function
	Eigen::Matrix3f rot;
	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			rot(i,j) = T.at<double>(i,j);
	Eigen::Vector3f euler_angles = rot.eulerAngles(2,1,0);
	double robot_yaw = euler_angles(0);
	geometry_msgs::Twist tw;
	error_phi = robot_configuration.pose_phi_ - robot_yaw;
	while (error_phi < -CV_PI*0.5)
		error_phi += CV_PI;
	while (error_phi > CV_PI*0.5)
		error_phi -= CV_PI;
	error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
	error_y = robot_configuration.pose_y_ - T.at<double>(1,3);

	std::cout << "error_x=" << error_x << "   error_y=" << error_y << "   error_phi=" << error_phi << std::endl;
	if (fabs(error_phi) > 0.03 || fabs(error_x) > 0.02 || fabs(error_y) > 0.02)
	{
		// control robot angle
		while(true)
		{
			if (!getTransform("checkerboard_reference_nav", "base_link", T))
				return false;
			Eigen::Matrix3f rot;
			for (int i=0; i<3; ++i)
				for (int j=0; j<3; ++j)
					rot(i,j) = T.at<double>(i,j);
			Eigen::Vector3f euler_angles = rot.eulerAngles(2,1,0);
			double robot_yaw = euler_angles(0);
			geometry_msgs::Twist tw;
			error_phi = robot_configuration.pose_phi_ - robot_yaw;
			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;
			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;
			tw.angular.z = std::min(0.05, error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// control position
		while(true)
		{
			if (!getTransform("checkerboard_reference_nav", "base_link", T))
				return false;
			geometry_msgs::Twist tw;
			error_x = robot_configuration.pose_x_ - T.at<double>(0,3);
			error_y = robot_configuration.pose_y_ - T.at<double>(1,3);
			if ((fabs(error_x) < 0.01 && fabs(error_y) < 0.01) || !ros::ok())
				break;
//			std::cout << "error_x: " << error_x << std::endl;
//			std::cout << "error_y: " << error_y << std::endl;
			tw.linear.x = std::min(0.05, error_x);
			tw.linear.y = std::min(0.05, error_y);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}

		// control robot angle
		while (true)
		{
			if (!getTransform("checkerboard_reference_nav", "base_link", T))
				return false;
			Eigen::Matrix3f rot;
			for (int i=0; i<3; ++i)
				for (int j=0; j<3; ++j)
					rot(i,j) = T.at<double>(i,j);
			Eigen::Vector3f euler_angles = rot.eulerAngles(2,1,0);
			double robot_yaw = euler_angles(0);
			geometry_msgs::Twist tw;
			error_phi = robot_configuration.pose_phi_ - robot_yaw;
			while (error_phi < -CV_PI*0.5)
				error_phi += CV_PI;
			while (error_phi > CV_PI*0.5)
				error_phi -= CV_PI;
			if (fabs(error_phi) < 0.02 || !ros::ok())
				break;
			tw.angular.z = std::min(0.05, error_phi);
			base_controller_.publish(tw);
			ros::Rate(20).sleep();
		}
	}

	std_msgs::Float64 msg;
	msg.data = robot_configuration.pan_angle_;
	pan_controller_.publish(msg);
	msg.data = robot_configuration.tilt_angle_;
	tilt_controller_.publish(msg);

	ros::Duration(3).sleep();

	std::cout << "Positioning successful: x=" << robot_configuration.pose_x_ << ", y=" << robot_configuration.pose_y_
			<< ", phi=" << robot_configuration.pose_phi_ << ", pan=" << robot_configuration.pan_angle_
			<< ", tilt=" << robot_configuration.tilt_angle_ << std::endl;

	return true;
}

bool CameraBaseCalibration::acquireCalibrationImages(int& image_width, int& image_height,
		std::vector< std::vector<cv::Point2f> >& points_2d_per_image, std::vector<cv::Mat>& T_base_to_checkerboard_vector,
		std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector, std::vector<cv::Mat>& T_camera_to_camera_optical_vector,
		const cv::Size pattern_size, const bool load_images)
{
	// capture images from different perspectives
	// todo: define pan/tilt unit positions and robot base locations relative to checkerboard
	std::vector<RobotConfiguration> robot_configurations;
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, 0.15, 0.25));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, 0.0, 0.3));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.15, 0.3));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.3, 0.3));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.5, 0.3));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, 0.15, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, 0.0, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.15, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.3, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.5, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, 0.15, -0.2));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, 0.0, -0.2));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.15, -0.2));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.35, -0.2));
	robot_configurations.push_back(RobotConfiguration(-1.5, -0.17, 0, -0.5, -0.2));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, 0.0, 0.2));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, -0.2, 0.2));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, -0.45, 0.2));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, 0.0, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, -0.2, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, -0.45, 0.05));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, 0.0, -0.15));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, -0.2, -0.15));
	robot_configurations.push_back(RobotConfiguration(-1.0, -0.17, 0, -0.45, -0.15));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, 0.0, 0.15));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, -0.15, 0.15));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, -0.35, 0.2));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, 0.0, 0.05));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, -0.15, 0.05));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, -0.35, 0.05));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, 0.0, -0.1));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, -0.15, -0.1));
	robot_configurations.push_back(RobotConfiguration(-0.85, -0.17, 0, -0.35, -0.1));

	const int number_images_to_capture = (int)robot_configurations.size();
	for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
	{
		if (!load_images)
			moveRobot(robot_configurations[image_counter]);

		// acquire image and extract checkerboard points
		std::vector<cv::Point2f> checkerboard_points_2d;
		int return_value = acquireCalibrationImage(image_width, image_height, checkerboard_points_2d, pattern_size, load_images, image_counter);
		if (return_value != 0)
			continue;

		// retrieve transformations
		cv::Mat T_base_to_checkerboard, T_torso_lower_to_torso_upper, T_camera_to_camera_optical;
		std::stringstream path;
		path << camera_calibration_path_ << image_counter << ".yml";
		if (load_images)
		{
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
			if (fs.isOpened())
			{
				fs["T_base_to_checkerboard"] >> T_base_to_checkerboard;
				fs["T_torso_lower_to_torso_upper"] >> T_torso_lower_to_torso_upper;
				fs["T_camera_to_camera_optical"] >> T_camera_to_camera_optical;
			}
			else
			{
				ROS_WARN("Could not read transformations from file '%s'.", path.str().c_str());
				continue;
			}
			fs.release();
		}
		else
		{
			bool result = true;
			result &= getTransform(base_frame_, checkerboard_frame_, T_base_to_checkerboard);
			result &= getTransform(torso_lower_frame_, torso_upper_frame_, T_torso_lower_to_torso_upper);
			result &= getTransform(camera_frame_, camera_optical_frame_, T_camera_to_camera_optical);

			if (result == false)
				continue;

			// save transforms to file
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
			if (fs.isOpened())
			{
				fs << "T_base_to_checkerboard" << T_base_to_checkerboard;
				fs << "T_torso_lower_to_torso_upper" << T_torso_lower_to_torso_upper;
				fs << "T_camera_to_camera_optical" << T_camera_to_camera_optical;
			}
			else
			{
				ROS_WARN("Could not write transformations to file '%s'.", path.str().c_str());
				continue;
			}
			fs.release();
		}

		points_2d_per_image.push_back(checkerboard_points_2d);
		T_base_to_checkerboard_vector.push_back(T_base_to_checkerboard);
		T_torso_lower_to_torso_upper_vector.push_back(T_torso_lower_to_torso_upper);
		T_camera_to_camera_optical_vector.push_back(T_camera_to_camera_optical);
		std::cout << "Captured perspectives: " << points_2d_per_image.size() << std::endl;
	}

	return true;
}

int CameraBaseCalibration::acquireCalibrationImage(int& image_width, int& image_height,
		std::vector<cv::Point2f>& checkerboard_points_2d, const cv::Size pattern_size, const bool load_images, int& image_counter)
{
	int return_value = 0;

	// acquire image
	cv::Mat image;
	if (load_images == false)
	{
		ros::Duration(3).sleep();
		capture_image_ = true;
		ros::spinOnce();
		ros::Duration(2).sleep();

		// retrieve image from camera
		{
			boost::mutex::scoped_lock lock(camera_data_mutex_);

			std::cout << "Time diff: " << (ros::Time::now() - latest_image_time_).toSec() << std::endl;

			if ((ros::Time::now() - latest_image_time_).toSec() < 20.0)
			{
				image = camera_image_.clone();
			}
			else
			{
				ROS_WARN("Did not receive camera images recently.");
				return -1;		// -1 = no fresh image available
			}
		}
	}
	else
	{
		// load image from file
		std::stringstream ss;
		ss << camera_calibration_path_ << image_counter;
		std::string image_name = ss.str() + ".png";
		image = cv::imread(image_name.c_str(), 0);
		if (image.empty())
			return -2;
	}
	image_width = image.cols;
	image_height = image.rows;

	// find pattern in image
	bool pattern_found = cv::findChessboardCorners(image, pattern_size, checkerboard_points_2d, cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_FILTER_QUADS);
	cv::Mat display = image.clone();
	cv::drawChessboardCorners(display, pattern_size, cv::Mat(checkerboard_points_2d), pattern_found);

	// display
	cv::imshow("image", display);
	bool capture_image = false;
	char key = cv::waitKey(10);
//	if (key == 'q')
//		return_value = 1;
//	else if (key == 'c')
//		capture_image = true;


	// collect 2d points
	if (checkerboard_points_2d.size() == pattern_size.height*pattern_size.width)
	{
		// check whether this perspective was already captured
		bool already_captured = false;
//		const double similarity_limit = 0.01*image_width*checkerboard_points_2d.size();
//		for (size_t i=0; i<points_2d_per_image.size(); ++i)
//			if (cv::norm(cv::Mat(checkerboard_points_2d), cv::Mat(points_2d_per_image[i]), cv::NORM_L2) < similarity_limit)
//			{
//				already_captured = true;
//				break;
//			}
////			const double similarity_limit = 0.02*fz_width*fz_points_2d.size();
////			for (size_t i=0; i<fz_points_2d_per_image.size(); ++i)
////				if (cv::norm(cv::Mat(fz_points_2d), cv::Mat(fz_points_2d_per_image[i]), cv::NORM_L2) < similarity_limit)
////				{
////					already_captured = true;
////					break;
////				}

		if (already_captured == false || capture_image == true)
		{
			// save images
			if (load_images == false)
			{
				std::stringstream ss;
				ss << camera_calibration_path_ << image_counter;
				std::string image_name = ss.str() + ".png";
				cv::imwrite(image_name.c_str(), image);
			}
		}
	}
	else
	{
		ROS_WARN("Not all checkerboard points have been observed.");
		return_value = -2;
	}

	return return_value;
}

void CameraBaseCalibration::computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const double chessboard_cell_size, const int number_images)
{
	// prepare chessboard 3d points
	pattern_points.clear();
	pattern_points.resize(1);
	pattern_points[0].resize(pattern_size.height*pattern_size.width);
	for (int v=0; v<pattern_size.height; ++v)
		for (int u=0; u<pattern_size.width; ++u)
			pattern_points[0][v*pattern_size.width+u] = cv::Point3f(u*chessboard_cell_size, v*chessboard_cell_size, 0.f);
	pattern_points.resize(number_images, pattern_points[0]);
}

void CameraBaseCalibration::intrinsicCalibration(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs)
{
	std::cout << "Intrinsic calibration started ..." << std::endl;
	K_ = cv::Mat::eye(3, 3, CV_64F);
	distortion_ = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(pattern_points, camera_points_2d_per_image, image_size, K_, distortion_, rvecs, tvecs);
	std::cout << "Intrinsic calibration:\nK:\n" << K_ << "\ndistortion:\n" << distortion_ << std::endl;
}

void CameraBaseCalibration::extrinsicCalibrationTorsoUpperToCamera(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_checkerboard_vector)
{
	// transform 3d chessboard points to respective coordinates systems (camera and torso_upper)
	std::vector<cv::Point3d> points_3d_camera, points_3d_torso_upper;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_upper_to_checkerboard = T_torso_lower_to_torso_upper_vector[i].inv() * T_base_to_torso_lower_.inv() * T_base_to_checkerboard_vector[i];
//		std::cout << "T_camera_to_checkerboard_vector[i]:\n" << T_camera_to_checkerboard_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_checkerboard:\n" << T_torso_upper_to_checkerboard << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_camera = T_camera_to_checkerboard_vector[i] * point;
			//std::cout << "point_camera=" << point_camera << std::endl;
			points_3d_camera.push_back(cv::Point3d(point_camera.at<double>(0), point_camera.at<double>(1), point_camera.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_upper = T_torso_upper_to_checkerboard * point;
			//std::cout << "point_torso_upper=" << point_torso_upper << std::endl;
			points_3d_torso_upper.push_back(cv::Point3d(point_torso_upper.at<double>(0), point_torso_upper.at<double>(1), point_torso_upper.at<double>(2)));
		}
	}

	// from: http://nghiaho.com/?page_id=671 : ‘A Method for Registration of 3-D Shapes’, by Besl and McKay, 1992.
	cv::Point3d centroid_torso_upper, centroid_camera;
	for (size_t i=0; i<points_3d_torso_upper.size(); ++i)
	{
		centroid_torso_upper += points_3d_torso_upper[i];
		centroid_camera += points_3d_camera[i];
	}
	centroid_torso_upper *= 1.0/(double)points_3d_torso_upper.size();
	centroid_camera *= 1.0/(double)points_3d_camera.size();

	cv::Mat M = cv::Mat::zeros(3,3,CV_64FC1);
	for (size_t i=0; i<points_3d_torso_upper.size(); ++i)
		M += cv::Mat(points_3d_camera[i] - centroid_camera)*cv::Mat(points_3d_torso_upper[i] - centroid_torso_upper).t();

	cv::Mat w, u, vt;
	cv::SVD::compute(M, w, u, vt, cv::SVD::FULL_UV);

	cv::Mat R = vt.t()*u.t();

	if (cv::determinant(R) < 0)
		for (int r=0; r<3; ++r)
			R.at<double>(r,2) *= -1;

	cv::Mat t = -R*cv::Mat(centroid_camera) + cv::Mat(centroid_torso_upper);

	std::cout << "R:\n" << R << "t:\n" << t << std::endl;

	T_torso_upper_to_camera_ = makeTransform(R, t);

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_torso_upper(new pcl::PointCloud<pcl::PointXYZ>);
//
//	// Fill in the CloudIn data
//	cloud_camera->width = points_3d_camera.size();
//	cloud_camera->height = 1;
//	cloud_camera->is_dense = false;
//	cloud_camera->points.resize(cloud_camera->width * cloud_camera->height);
//	for (size_t i = 0; i < cloud_camera->points.size(); ++i)
//	{
//		cloud_camera->points[i].x = points_3d_camera[i].x;
//		cloud_camera->points[i].y = points_3d_camera[i].y;
//		cloud_camera->points[i].z = points_3d_camera[i].z;
//	}
//
//	// Fill in the CloudIn data
//	cloud_torso_upper->width = points_3d_torso_upper.size();
//	cloud_torso_upper->height = 1;
//	cloud_torso_upper->is_dense = false;
//	cloud_torso_upper->points.resize(cloud_torso_upper->width * cloud_torso_upper->height);
//	for (size_t i = 0; i < cloud_torso_upper->points.size(); ++i)
//	{
//		cloud_torso_upper->points[i].x = points_3d_torso_upper[i].x;
//		cloud_torso_upper->points[i].y = points_3d_torso_upper[i].y;
//		cloud_torso_upper->points[i].z = points_3d_torso_upper[i].z;
//	}
//
//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//	icp.setInputSource(cloud_camera);
//	icp.setInputTarget(cloud_torso_upper);
//	pcl::PointCloud<pcl::PointXYZ> final;
//	icp.align(final);
//	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//	std::cout << icp.getFinalTransformation() << std::endl;
//
//	Eigen::Matrix4f T = icp.getFinalTransformation();
//	for (int r=0; r<4; ++r)
//		for (int c=0; c<4; ++c)
//			T_torso_upper_to_camera_.at<double>(r,c) = T(r,c);

//	// estimate transform
//	cv::Mat transform, inliers;
//	cv::estimateAffine3D(cv::Mat(points_3d_camera), cv::Mat(points_3d_torso_upper), transform, inliers);
//	T_torso_upper_to_camera_ = transform;
//	cv::Mat last_row = cv::Mat(cv::Vec4d(0., 0., 0., 1.)).t();
//	T_torso_upper_to_camera_.push_back(last_row);
}

void CameraBaseCalibration::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_checkerboard_vector)
{
	// transform 3d chessboard points to respective coordinates systems (base and torso_lower)
	std::vector<cv::Point3d> points_3d_base, points_3d_torso_lower;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_lower_to_checkerboard = T_torso_lower_to_torso_upper_vector[i] * T_torso_upper_to_camera_ * T_camera_to_checkerboard_vector[i];
//		std::cout << "T_base_to_checkerboard_vector[" << i << "]:\n" << T_base_to_checkerboard_vector[i] << std::endl;
//		std::cout << "T_torso_lower_to_checkerboard:\n" << T_torso_lower_to_checkerboard << std::endl;
//		std::cout << "T_torso_lower_to_torso_upper_vector[i]:\n" << T_torso_lower_to_torso_upper_vector[i] << std::endl;
//		std::cout << "T_torso_upper_to_camera_:\n" << T_torso_upper_to_camera_ << std::endl;
//		std::cout << "T_camera_to_checkerboard_vector[i]:\n" << T_camera_to_checkerboard_vector[i] << std::endl;
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat(cv::Vec4d(pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0));

			// to camera coordinate system
			cv::Mat point_base = T_base_to_checkerboard_vector[i] * point;
			//std::cout << "point_base: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_base.at<double>(0,0) <<", "<< point_base.at<double>(1,0) << ", " << point_base.at<double>(2,0) << std::endl;
			points_3d_base.push_back(cv::Point3d(point_base.at<double>(0), point_base.at<double>(1), point_base.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_lower = T_torso_lower_to_checkerboard * point;
			//std::cout << "point_torso_lower: " << pattern_points_3d[i][j].x <<", "<< pattern_points_3d[i][j].y <<", "<< pattern_points_3d[i][j].z << " --> " << point_torso_lower.at<double>(0) <<", "<< point_torso_lower.at<double>(1) << ", " << point_torso_lower.at<double>(2) << std::endl;
			points_3d_torso_lower.push_back(cv::Point3d(point_torso_lower.at<double>(0), point_torso_lower.at<double>(1), point_torso_lower.at<double>(2)));
		}
	}

	// from: http://nghiaho.com/?page_id=671 : ‘A Method for Registration of 3-D Shapes’, by Besl and McKay, 1992.
	cv::Point3d centroid_base, centroid_torso_lower;
	for (size_t i=0; i<points_3d_base.size(); ++i)
	{
		centroid_base += points_3d_base[i];
		centroid_torso_lower += points_3d_torso_lower[i];
	}
	centroid_base *= 1.0/(double)points_3d_base.size();
	centroid_torso_lower *= 1.0/(double)points_3d_torso_lower.size();

	cv::Mat M = cv::Mat::zeros(3,3,CV_64FC1);
	for (size_t i=0; i<points_3d_base.size(); ++i)
		M += cv::Mat(points_3d_torso_lower[i] - centroid_torso_lower)*cv::Mat(points_3d_base[i] - centroid_base).t();

	cv::Mat w, u, vt;
	cv::SVD::compute(M, w, u, vt, cv::SVD::FULL_UV);

	cv::Mat R = vt.t()*u.t();

	if (cv::determinant(R) < 0)
		for (int r=0; r<3; ++r)
			R.at<double>(r,2) *= -1;

	cv::Mat t = -R*cv::Mat(centroid_torso_lower) + cv::Mat(centroid_base);

	std::cout << "R:\n" << R << "t:\n" << t << std::endl;

	T_base_to_torso_lower_ = makeTransform(R, t);



//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_torso_lower(new pcl::PointCloud<pcl::PointXYZ>);
//
//	// Fill in the CloudIn data
//	cloud_base->width = points_3d_base.size();
//	cloud_base->height = 1;
//	cloud_base->is_dense = false;
//	cloud_base->points.resize(cloud_base->width * cloud_base->height);
//	for (size_t i = 0; i < cloud_base->points.size(); ++i)
//	{
//		cloud_base->points[i].x = points_3d_base[i].x;
//		cloud_base->points[i].y = points_3d_base[i].y;
//		cloud_base->points[i].z = points_3d_base[i].z;
//	}
//
//	// Fill in the CloudIn data
//	cloud_torso_lower->width = points_3d_torso_lower.size();
//	cloud_torso_lower->height = 1;
//	cloud_torso_lower->is_dense = false;
//	cloud_torso_lower->points.resize(cloud_torso_lower->width * cloud_torso_lower->height);
//	for (size_t i = 0; i < cloud_torso_lower->points.size(); ++i)
//	{
//		cloud_torso_lower->points[i].x = points_3d_torso_lower[i].x;
//		cloud_torso_lower->points[i].y = points_3d_torso_lower[i].y;
//		cloud_torso_lower->points[i].z = points_3d_torso_lower[i].z;
//	}
//
//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//	icp.setInputSource(cloud_base);
//	icp.setInputTarget(cloud_torso_lower);
//	pcl::PointCloud<pcl::PointXYZ> final;
//	icp.align(final);
//	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//	std::cout << icp.getFinalTransformation() << std::endl;
//
//	Eigen::Matrix4f T = icp.getFinalTransformation();
//	for (int r=0; r<4; ++r)
//		for (int c=0; c<4; ++c)
//			T_base_to_torso_lower_.at<double>(r,c) = T(r,c);

//	// estimate transform
//	cv::Mat transform, inliers;
//	cv::estimateAffine3D(cv::Mat(points_3d_torso_lower), cv::Mat(points_3d_base), transform, inliers);
//	T_base_to_torso_lower_ = transform;
//	cv::Mat last_row = cv::Mat(cv::Vec4d(0., 0., 0., 1.)).t();
//	T_base_to_torso_lower_.push_back(last_row);
}

bool CameraBaseCalibration::saveCalibration()
{
	bool success = true;

	// save calibration
	std::string filename = camera_calibration_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "K" << K_;
		fs << "distortion" << distortion_;
		fs << "T_base_to_torso_lower" << T_base_to_torso_lower_;
		fs << "T_torso_upper_to_camera" << T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibration::saveCalibration: Could not write calibration to file.";
		success = false;
	}
	fs.release();

	return success;
}

bool CameraBaseCalibration::loadCalibration()
{
	bool success = true;

	// load calibration
	std::string filename = camera_calibration_path_ + "camera_calibration.yml";
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["K"] >> K_;
		fs["distortion"] >> distortion_;
		fs["T_base_to_torso_lower"] >> T_base_to_torso_lower_;
		fs["T_torso_upper_to_camera"] >> T_torso_upper_to_camera_;
	}
	else
	{
		std::cout << "Error: CameraBaseCalibration::loadCalibration: Could not read calibration from file.";
		success = false;
	}
	fs.release();

	calibrated_ = true;

	return success;
}

void CameraBaseCalibration::getCalibration(cv::Mat& K, cv::Mat& distortion, cv::Mat& T_base_to_torso_lower, cv::Mat& T_torso_upper_to_camera)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: CameraBaseCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	K = K_.clone();
	distortion = distortion_.clone();
	T_base_to_torso_lower = T_base_to_torso_lower_.clone();
	T_torso_upper_to_camera = T_torso_upper_to_camera_.clone();
}

void CameraBaseCalibration::undistort(const cv::Mat& image, cv::Mat& image_undistorted)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::undistort(image, image_undistorted, K_, distortion_);
}
