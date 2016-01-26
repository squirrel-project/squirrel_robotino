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

#include <pcl/io/pcd_io.h>

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
			tilt_controller_command_("/tilt_controller/command"), pan_controller_command_("/tilt_controller/command")
{
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "colorimage_in", 1);
	color_image_sub_.registerCallback(boost::bind(&CameraBaseCalibration::imageCallback, this, _1));

	tilt_controller_ = node_handle_.advertise<std_msgs::Float64>(tilt_controller_command_, 1, false);
	pan_controller_ = node_handle_.advertise<std_msgs::Float64>(pan_controller_command_, 1, false);

	// todo: parameters
	torso_lower_frame_ = "pan_link";
	torso_upper_frame_ = "tilt_link";
	camera_frame_ = "kinect_link";
	base_frame_ = "base_link";
	checkerboard_frame_ = "checkerboard";

	chessboard_cell_size_ = 0.05;
	chessboard_pattern_size_ = cv::Size(6,4);

	// todo: set good initial parameters
//	T_base_to_torso_lower_;
//	T_torso_upper_to_camera_;
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
		image_ptr = cv_bridge::toCvShare(image_msg, image_msg->encoding);
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

	// read image
	cv_bridge::CvImageConstPtr color_image_ptr;
	if (convertImageMessageToMat(color_image_msg, color_image_ptr, camera_image_) == false)
		return;

	latest_image_time_ = color_image_msg->header.stamp;
}

bool CameraBaseCalibration::calibrateCameraToBase(const bool load_images)
{
	// setup storage folder
	int return_value = system("mkdir -p robotino_calibration/camera_calibration");

	// acquire images
	int image_width=0, image_height=0;
	std::vector< std::vector<cv::Point2f> > points_2d_per_image;
	std::vector<cv::Mat> T_base_to_checkerboard_vector;
	std::vector<cv::Mat> T_torso_lower_to_torso_upper_vector;
	acquireCalibrationImages(image_width, image_height, points_2d_per_image, T_base_to_checkerboard_vector,
			T_torso_lower_to_torso_upper_vector, chessboard_pattern_size_, load_images);

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
		cv::Mat T_camera_to_checkerboard = makeTransform(R, tvecs[i]);
		T_camera_to_checkerboard_vector.push_back(T_camera_to_checkerboard);
	}

	// extrinsic calibration between base and torso_lower as well ass torso_upper and camera
	for (int i=0; i<5; ++i)
	{
		std::cout << "Extrinsic optimization run " << i << ":" << std::endl;
		extrinsicCalibrationBaseToTorsoLower(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
		std::cout << "T_base_to_torso_lower:\n" << T_base_to_torso_lower_ << std::endl;
		extrinsicCalibrationTorsoUpperToCamera(pattern_points_3d, T_base_to_checkerboard_vector, T_torso_lower_to_torso_upper_vector, T_camera_to_checkerboard_vector);
		std::cout << "T_torso_upper_to_camera:\n" << T_torso_upper_to_camera_ << std::endl;
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
		std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}

	return true;
}

bool CameraBaseCalibration::acquireCalibrationImages(int& image_width, int& image_height,
		std::vector< std::vector<cv::Point2f> >& points_2d_per_image, std::vector<cv::Mat>& T_base_to_checkerboard_vector,
		std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		const cv::Size pattern_size, const bool load_images)
{
	// capture images from different perspectives
	const int number_images_to_capture = 10;
	// todo: define pan/tilt unit positions and robot base locations relative to checkerboard
	for (int image_counter = 0; image_counter < number_images_to_capture; ++image_counter)
	{
		// todo: drive robot to position and turn pan/tilt unit
//		tilt_controller_;
//		pan_controller_;

		// acquire image and extract checkerboard points
		std::vector<cv::Point2f> checkerboard_points_2d;
		int return_value = acquireCalibrationImage(image_width, image_height, checkerboard_points_2d, pattern_size, load_images, image_counter);
		if (return_value != 0)
			continue;

		// retrieve transformations
		cv::Mat T_base_to_checkerboard, T_torso_lower_to_torso_upper;
		std::stringstream path;
		path << camera_calibration_path_ << image_counter << ".yml";
		if (load_images)
		{
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::READ);
			if (fs.isOpened())
			{
				fs["T_base_to_checkerboard"] >> T_base_to_checkerboard;
				fs["T_torso_lower_to_torso_upper"] >> T_torso_lower_to_torso_upper;
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
			result &= getTransform(checkerboard_frame_, base_frame_, T_base_to_checkerboard);
			result &= getTransform(torso_upper_frame_, torso_lower_frame_, T_torso_lower_to_torso_upper);
			if (result == false)
				continue;

			// save transforms to file
			cv::FileStorage fs(path.str().c_str(), cv::FileStorage::WRITE);
			if (fs.isOpened())
			{
				fs << "T_base_to_checkerboard" << T_base_to_checkerboard;
				fs << "T_torso_lower_to_torso_upper" << T_torso_lower_to_torso_upper;
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
		// retrieve image from camera
		boost::mutex::scoped_lock lock(camera_data_mutex_);

		if ((ros::Time::now() - latest_image_time_).toSec() < 2.0)
		{
			image = camera_image_.clone();
		}
		else
		{
			ROS_WARN("Did not receive camera images recently.");
			return -1;		// -1 = no fresh image available
		}
	}
	else
	{
		// load image from file
		std::stringstream ss;
		ss << camera_calibration_path_ << image_counter;
		std::string image_name = ss.str() + ".png";
		image = cv::imread(image_name.c_str(), 0);
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

		if (already_captured == false || capture_image == true || load_images == true)
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
	std::vector<cv::Point3f> points_3d_camera, points_3d_torso_upper;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_upper_to_checkerboard = T_torso_lower_to_torso_upper_vector[i].inv() * T_base_to_torso_lower_.inv() * T_base_to_checkerboard_vector[i];
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat_<double>(4,1) << (pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0);

			// to camera coordinate system
			cv::Mat point_camera = T_camera_to_checkerboard_vector[i] * point;
			points_3d_camera.push_back(cv::Point3f(point_camera.at<double>(0), point_camera.at<double>(1), point_camera.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_upper = T_torso_upper_to_checkerboard * point;
			points_3d_torso_upper.push_back(cv::Point3f(point_torso_upper.at<double>(0), point_torso_upper.at<double>(1), point_torso_upper.at<double>(2)));
		}
	}

	// estimate transform
	cv::Mat transform, inliers;
	cv::estimateAffine3D(cv::Mat(points_3d_torso_upper), cv::Mat(points_3d_camera), transform, inliers);
	T_torso_upper_to_camera_ = transform;
	cv::Mat last_row = cv::Mat_<double>(1,4) << (0., 0., 0., 1.);
	T_torso_upper_to_camera_.push_back(last_row);
}

void CameraBaseCalibration::extrinsicCalibrationBaseToTorsoLower(std::vector< std::vector<cv::Point3f> >& pattern_points_3d,
		std::vector<cv::Mat>& T_base_to_checkerboard_vector, std::vector<cv::Mat>& T_torso_lower_to_torso_upper_vector,
		std::vector<cv::Mat>& T_camera_to_checkerboard_vector)
{
	// transform 3d chessboard points to respective coordinates systems (base and torso_lower)
	std::vector<cv::Point3f> points_3d_base, points_3d_torso_lower;
	for (size_t i=0; i<pattern_points_3d.size(); ++i)
	{
		cv::Mat T_torso_lower_to_checkerboard = T_torso_lower_to_torso_upper_vector[i] * T_torso_upper_to_camera_ * T_camera_to_checkerboard_vector[i];
		for (size_t j=0; j<pattern_points_3d[i].size(); ++j)
		{
			cv::Mat point = cv::Mat_<double>(4,1) << (pattern_points_3d[i][j].x, pattern_points_3d[i][j].y, pattern_points_3d[i][j].z, 1.0);

			// to camera coordinate system
			cv::Mat point_base = T_base_to_checkerboard_vector[i] * point;
			points_3d_base.push_back(cv::Point3f(point_base.at<double>(0), point_base.at<double>(1), point_base.at<double>(2)));

			// to torso_upper coordinate
			cv::Mat point_torso_lower = T_torso_lower_to_checkerboard * point;
			points_3d_torso_lower.push_back(cv::Point3f(point_torso_lower.at<double>(0), point_torso_lower.at<double>(1), point_torso_lower.at<double>(2)));
		}
	}

	// estimate transform
	cv::Mat transform, inliers;
	cv::estimateAffine3D(cv::Mat(points_3d_base), cv::Mat(points_3d_torso_lower), transform, inliers);
	T_base_to_torso_lower_ = transform;
	cv::Mat last_row = cv::Mat_<double>(1,4) << (0., 0., 0., 1.);
	T_base_to_torso_lower_.push_back(last_row);
}

//void CameraBaseCalibration::extrinsicCalibration(const std::vector<cv::Point3f>& fz_points_3d_vector, const std::vector<cv::Point2f>& jai_points_2d_vector)
//{
//	// clean list of 3d points from invalid measurements
//	std::vector<cv::Point3f> fz_points_3d_cleaned;
//	std::vector<cv::Point2f> jai_points_2d_cleaned;
//	for (size_t i=0; i<fz_points_3d_vector.size(); ++i)
//	{
//		if (!(fz_points_3d_vector[i].x==0.f && fz_points_3d_vector[i].y==0.f && fz_points_3d_vector[i].z==0.f) || fz_points_3d_vector[i].z!=fz_points_3d_vector[i].z)
//		{
//			fz_points_3d_cleaned.push_back(fz_points_3d_vector[i]);
//			jai_points_2d_cleaned.push_back(jai_points_2d_vector[i]);
//		}
//	}
//
//	// extrinsic calibration from fz to jai
//	cv::Mat rvec;
//	cv::solvePnP(fz_points_3d_cleaned, jai_points_2d_cleaned, K_, distortion_, rvec, t_, false, cv::ITERATIVE);
//	cv::Rodrigues(rvec, R_);
//	std::cout << "solvePnP: Extrinsinc calibration:\nR:\n" << R_ << "\nt:\n" << t_ << std::endl;
//}
//
//void CameraBaseCalibration::cameraRobotCalibration(const std::vector<cv::Mat>& rvecs_jai, const std::vector<cv::Mat>& tvecs_jai)
//{
//	// arm-camera calibration
//	// x, y, z, w, p, r (= yaw, pitch, roll with 1. rotation = roll around z, 2. rotation = pitch around y', 3. rotation around x'')
//	cv::Mat arm_positions = (cv::Mat_<float>(39,6) <<
//			32.534,	1391.635,	436.573,	179.833,	-0.005,	-119.501,
//			2.678, 	1391.616, 	436.656, 	179.830, 	-0.002, 	-119.500,
//			-30.383, 	1391.613, 	436.734, 	179.828, 	0.0, 	-115.375,
//			-30.354, 	1416.441, 	436.826, 	179.824, 	0.0, 	-123.394,
//			14.688, 	1409.591, 	436.908, 	179.821, 	0.003, 	-123.393,
//			47.042, 	1409.565, 	436.979, 	179.819, 	0.005, 	-123.392,
//			32.214, 	1324.595, 	437.057, 	179.816, 	0.008, 	-123.392,
//			0.259, 	1324.575, 	437.117, 	179.813, 	0.010, 	-123.393,
//			-18.412, 	1324.571, 	437.180, 	179.811, 	0.011, 	-126.529,
//			-18.558, 	1361.942, 	437.416, 	-174.848, 	-7.158, 	-126.864,
//			23.253, 	1408.345, 	437.497, 	-179.405, 	-10.554, 	-126.740,
//			-10.261, 	1423.790, 	437.589, 	-169.205, 	-2.724, 	-120.907,
//			-14.834, 	1353.210, 	437.828, 	-175.886, 	8.687, 	-114.118,
//			-14.842, 	1353.177, 	724.456, 	-179.104, 	-1.929, 	-120.782,
//			-153.997, 	1374.504, 	724.623, 	-179.114, 	-1.930, 	-120.780,
//			189.747, 	1360.784, 	724.693, 	-179.122, 	-1.932, 	-120.780,
//			189.769, 	1507.976, 	724.787, 	-179.128, 	-1.931, 	-120.780,
//			-0.612, 	1507.938, 	724.906, 	-179.131, 	-1.927, 	-120.780,
//			-165.797, 	1507.902, 	725.009, 	-179.137, 	-1.924, 	-120.782,
//			-165.744, 	1196.436, 	725.050, 	-179.138, 	-1.923, 	-120.782,
//			18.52, 	1196.407, 	725.085, 	-179.144, 	-1.926, 	-120.782,
//			188.480, 	1196.395, 	725.118, 	178.739, 	1.616, 	-120.779,
//			141.716, 	1253.977, 	725.170, 	-176.572, 	-6.232, 	-117.108,
//			112.132, 	1440.772, 	725.236, 	-177.507, 	-8.927, 	-127.909,
//			-123.648, 	1440.739, 	725.330, 	-168.315, 	-1.664, 	-128.759,
//			-123.585, 	1254.576, 	725.438, 	-171.849, 	4.523, 	-114.970,
//			-236.631, 	1195.501, 	878.487, 	-177.723, 	0.000, 	-120.557,
//			-14.921, 	1191.495, 	878.542, 	-177.731, 	-0.002, 	-120.556,
//			198.597, 	1191.471, 	849.785, 	174.825, 	1.025, 	-120.864,
//			198.590, 	1337.231, 	849.844, 	174.818, 	1.024, 	-120.864,
//			-7.679, 	1337.192, 	849.915, 	174.810, 	1.024, 	-120.862,
//			-201.665, 	1337.163, 	849.988, 	178.435, 	3.185, 	-120.731,
//			-201.657, 	1501.781, 	850.069, 	178.432, 	3.187, 	-120.731,
//			21.280, 	1501.731, 	850.156, 	178.429, 	3.192, 	-120.728,
//			172.489, 	1501.685, 	850.231, 	178.423, 	3.193, 	-120.727,
//			188.268, 	1501.638, 	850.321, 	167.294, 	-3.714, 	-115.901,
//			-188.132, 	1501.535, 	850.490, 	-171.924, 	6.441, 	-112.471,
//			-188.130, 	1302.978, 	850.634, 	-172.230, 	6.821, 	-124.221,
//			141.221, 	1253.279, 	850.679, 	174.824, 	-2.199, 	-129.705);
//
//	Eigen::Quaterniond avg_rotation(0,0,0,0);
//	Eigen::Vector3d avg_translation(0,0,0);
//	for (size_t index=0; index<rvecs_jai.size(); ++index)
//	{
//		std::cout << "\n-----" << index << "-----\n";
//		// todo: remove if TCP is defined and TCP coordinates are provided
//		// transform J6 to TCP
//		// todo: dynamically obtain TCP
//		double j6_to_tcp_roll = -32./180. * CV_PI - 90./180. * CV_PI;
//		double j6_to_tcp_pitch = 180./180. * CV_PI;
//		double j6_to_tcp_z = 0.2671;		// checkerboard height not included
//		cv::Mat T_j6_to_tcp = makeTransform(rotationMatrixFromRPY(j6_to_tcp_roll, j6_to_tcp_pitch, 0), cv::Mat(cv::Vec3d(0, 0, j6_to_tcp_z)));
//		//std::cout << "\n T_j6_to_tcp:\n" << T_j6_to_tcp << std::endl;
//
//		// transform world to j6
//		cv::Mat T_world_to_j6 = makeTransform(rotationMatrixFromRPY(arm_positions.at<float>(index,5)/180.*CV_PI, arm_positions.at<float>(index,4)/180.*CV_PI, arm_positions.at<float>(index,3)/180.*CV_PI), cv::Mat(cv::Vec3d(0.001*arm_positions.at<float>(index,0), 0.001*arm_positions.at<float>(index,1), 0.001*arm_positions.at<float>(index,2))));
//		//std::cout << "\n T_world_to_j6:\n" << T_world_to_j6 << std::endl;
//
//		// transform world to tcp
//		cv::Mat T_world_to_tcp = T_world_to_j6 * T_j6_to_tcp;
//		//std::cout << "\n T_world_to_tcp:\n" << T_world_to_tcp << std::endl;
//
//		// transform tcp to checkerboard
//		cv::Mat T_tcp_to_checkerboard = makeTransform(cv::Mat::eye(3,3,CV_64FC1), cv::Mat(cv::Vec3d(-0.125, -0.075, -0.006)));
//
//		cv::Mat T_world_to_checkerboard = T_world_to_tcp * T_tcp_to_checkerboard;
//		//std::cout << "\n T_world_to_checkerboard:\n" << T_world_to_checkerboard << std::endl;
//
//		// transform checkerboard to jai camera per rvecs_jai, tvecs_jai,
//		cv::Mat R, t;
//		cv::Rodrigues(rvecs_jai[index], R);
//		t = -R.t()*tvecs_jai[index];
//		cv::Mat T_checkerboard_to_jai = makeTransform(R.t(), t);
//		//std::cout << "\n T_checkerboard_to_jai:\n" << T_checkerboard_to_jai << std::endl;
//
//		// transform jai camera to fz camera
//		cv::Mat T_jai_to_fz = makeTransform(R_, t_);
//		//std::cout << "\n T_jai_to_fz:\n" << T_jai_to_fz << std::endl;
//
//		cv::Mat T_world_fz = T_world_to_checkerboard*T_checkerboard_to_jai*T_jai_to_fz;
//		std::cout << "\n T_world_fz:\n" << T_world_fz << std::endl;
//
//		Eigen::Matrix3d m;
//		for (int i=0;i<3;++i)
//			for (int j=0;j<3;++j)
//				m(i,j) = T_world_fz.at<double>(i,j);
//		Eigen::Quaterniond q(m);
//		avg_rotation.w() += q.w();
//		avg_rotation.x() += q.x();
//		avg_rotation.y() += q.y();
//		avg_rotation.z() += q.z();
//		for (int i=0; i<3; ++i)
//			avg_translation(i) += T_world_fz.at<double>(i,3);
//	}
//	float number = rvecs_jai.size();
//	avg_rotation.w() /= number;
//	avg_rotation.x() /= number;
//	avg_rotation.y() /= number;
//	avg_rotation.z() /= number;
//	Eigen::Matrix3d m = avg_rotation.matrix();
//	R_world_to_fz_.create(3,3,CV_64FC1);
//	for (int i=0;i<3;++i)
//		for (int j=0;j<3;++j)
//			R_world_to_fz_.at<double>(i,j) = m(i,j);
//	avg_translation /= number;
//	t_world_to_fz_.create(3,1,CV_64FC1);
//	for (int i=0; i<3; ++i)
//		t_world_to_fz_.at<double>(i,0) = avg_translation(i);
//
//	std::cout << "\n-------------------------\nR_world_to_fz:\n" << R_world_to_fz_ << "\nt_world_to_fz:\n" << t_world_to_fz_ << std::endl;
//}

//bool CameraBaseCalibration::testCalibration()
//{
//	const cv::Size pattern_size(6,4);
//	int black_white_threshold = 43;
//
//	// setup cameras
//	bool success = true;
//	//success &= openCameras();
//	success &= fotonic_c70_.setParametersCalibration(1, 40);
//	jai_go_5000_.setFeatureFloat("AcquisitionFrameRate", 40.0);
//	//success &= startCameras();
//	if (success == false)
//		return success;
//
//	// test calibration
//	bool running = true;
//	while (running == true)
//	{
//		// retrieve images from cameras
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr fz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//		cv::Mat fz_image, fz_ir_image, jai_image;
//		success &= jai_go_5000_.grabFrame(jai_image);
//		success &= fotonic_c70_.grabFrame(fz_cloud, fz_image, &fz_ir_image);
//		cv::normalize(fz_ir_image, fz_ir_image, 0., 255., cv::NORM_MINMAX);
//		cv::threshold(fz_ir_image, fz_ir_image, black_white_threshold, 255., cv::THRESH_BINARY);
//
//		// find calibration pattern 3d points from fz sensor
//		std::vector<cv::Point2f> fz_points_2d;
//		bool pattern_found = cv::findChessboardCorners(fz_ir_image, pattern_size, fz_points_2d, cv::CALIB_CB_FILTER_QUADS);
//		cv::Mat display_fz;
//		cv::cvtColor(fz_ir_image, display_fz, CV_GRAY2BGR);
//		cv::drawChessboardCorners(display_fz, pattern_size, cv::Mat(fz_points_2d), pattern_found);
//		cv::Mat display_jai, temp;
//		cv::undistort(jai_image, temp, K_, distortion_);
//		cv::cvtColor(temp, display_jai, CV_GRAY2BGR);
//
//		// convert all xyz coordinates of fz sensor to jai image coordinates
//		if (pattern_found == true)
//		{
//			std::vector< cv::Point3f > fz_points_3d_test(fz_points_2d.size());
//			for (size_t i=0; i<fz_points_2d.size(); ++i)
//			{
//				fz_points_3d_test[i] = getMean3DCoordinate(fz_cloud, fz_points_2d[i].x, fz_points_2d[i].y);
//				cv::Point3d mean_point = cv::Point3d(fz_points_3d_test[i].x, fz_points_3d_test[i].y, fz_points_3d_test[i].z);
////				pcl::PointXYZRGB& point = fz_cloud->at(fz_points_2d[i].x, fz_points_2d[i].y);
////				fz_points_3d_test[i] = cv::Point3f(point.x, point.y, point.z);
//				cv::Mat point_jai_mat = K_ * (R_ * cv::Mat(mean_point) + t_);
//				cv::circle(display_jai, cv::Point(point_jai_mat.at<double>(0)/point_jai_mat.at<double>(2), point_jai_mat.at<double>(1)/point_jai_mat.at<double>(2)), 4, CV_RGB(0,255,0), 2);
//			}
//			cv::Mat rvec;
//			cv::Rodrigues(R_, rvec);
//			std::vector< cv::Point2f> jai_points_projected;
//			cv::projectPoints(fz_points_3d_test, rvec, t_, K_, distortion_, jai_points_projected);
//			for (size_t i=0; i<jai_points_projected.size(); ++i)
//				cv::circle(display_jai, cv::Point(jai_points_projected[i].x, jai_points_projected[i].y), 4, CV_RGB(0,0,255), 2);
//		}
//
//		// display
//		cv::imshow("fz_ir_image", display_fz);
//		cv::imshow("jai_image", display_jai);
//		char key = cv::waitKey(10);
//		if (key == 'q')
//			running = false;
//	}
//
//	// clean up
//	//stopCameras();
//	//closeCameras();
//
//	return success;
//}

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

void CameraBaseCalibration::getCalibration(cv::Mat& K_jai, cv::Mat& distortion_jai, cv::Mat& R, cv::Mat& t)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	K_jai = K_.clone();
	distortion_jai = distortion_.clone();
	R = R_.clone();
	t = t_.clone();
}

void CameraBaseCalibration::getCameraWorldPose(cv::Mat& R_world_to_fz, cv::Mat& t_world_to_fz)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	R_world_to_fz = R_world_to_fz_.clone();
	t_world_to_fz = t_world_to_fz_.clone();
}

void CameraBaseCalibration::projectFz3dToJai2d(const std::vector<cv::Point3f>& fz_points_3d, std::vector<cv::Point2f>& jai_points_projected)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::Mat rvec;
	cv::Rodrigues(R_, rvec);
	cv::projectPoints(fz_points_3d, rvec, t_, K_, distortion_, jai_points_projected);
}

void CameraBaseCalibration::undistortJai(const cv::Mat& jai_image, cv::Mat& jai_image_undistorted)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::undistort(jai_image, jai_image_undistorted, K_, distortion_);
}

cv::Point3f CameraBaseCalibration::getMean3DCoordinate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int x, const int y)
{
	cv::Point3f mean_point(0,0,0);
	int num_avg_points = 0;
	for (int dv=-1; dv<=1; ++dv)
	{
		for (int du=-1; du<=1; ++du)
		{
			pcl::PointXYZRGB& point = cloud->at(x+du, y+dv);
			if (!((point.x==0.f && point.y==0.f && point.z==0.f) || point.z!=point.z))
			{
				mean_point += cv::Point3f(point.x, point.y, point.z);
				num_avg_points++;
			}
		}
	}
	mean_point *= (num_avg_points==0 ? 0 : 1./(double)num_avg_points);

	return mean_point;
}
