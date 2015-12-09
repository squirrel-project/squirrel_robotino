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

#include <robotino_calibration/hand_camera_calibration.h>

#include <opencv/highgui.h>
#include <pcl/io/pcd_io.h>


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

HandCameraCalibration::HandCameraCalibration()
{
}

HandCameraCalibration::~HandCameraCalibration();
{
}


bool HandCameraCalibration::calibrateHandToCameraExtrinsicOnly(const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images)
{
	// setup storage folder
	system("mkdir -p robotino_calibration");

	// acquire images
	int jai_width=0, jai_height=0;
	int fz_width=0, fz_height=0;
	std::vector<cv::Point2f> jai_points_2d_vector;
	std::vector<cv::Point3f> fz_points_3d_vector;
	std::vector< std::vector<cv::Point2f> > jai_points_2d_per_image;
	std::vector< std::vector<cv::Point2f> > fz_points_2d_per_image;
	acquireCalibrationImages(jai_width, jai_height, fz_width, fz_height, jai_points_2d_vector, fz_points_3d_vector, jai_points_2d_per_image, fz_points_2d_per_image, pattern_size, black_white_threshold, load_images, num_images);

	// prepare chessboard 3d points
	std::vector< std::vector<cv::Point3f> > pattern_points;
	computeCheckerboard3dPoints(pattern_points, pattern_size, jai_points_2d_per_image.size());

	// intrinsic calibration: jai go 5000
	std::vector<cv::Mat> rvecs_jai, tvecs_jai;
	intrinsicCalibrationJai(pattern_points, jai_points_2d_per_image, cv::Size(jai_width, jai_height), rvecs_jai, tvecs_jai);


	// camera-robot calibration
	cameraRobotCalibration(rvecs_jai, tvecs_jai);

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return success;
}


bool HandCameraCalibration::calibrateHandToCamera(const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images)
{
	// setup storage folder
	system("mkdir -p robotino_calibration");

	// acquire images
	int jai_width=0, jai_height=0;
	int fz_width=0, fz_height=0;
	std::vector<cv::Point2f> jai_points_2d_vector;
	std::vector<cv::Point3f> fz_points_3d_vector;
	std::vector< std::vector<cv::Point2f> > jai_points_2d_per_image;
	std::vector< std::vector<cv::Point2f> > fz_points_2d_per_image;
	acquireCalibrationImages(jai_width, jai_height, fz_width, fz_height, jai_points_2d_vector, fz_points_3d_vector, jai_points_2d_per_image, fz_points_2d_per_image, pattern_size, black_white_threshold, load_images, num_images);

	// prepare chessboard 3d points
	std::vector< std::vector<cv::Point3f> > pattern_points;
	computeCheckerboard3dPoints(pattern_points, pattern_size, jai_points_2d_per_image.size());

	// intrinsic calibration: jai go 5000
	std::vector<cv::Mat> rvecs_jai, tvecs_jai;
	intrinsicCalibrationJai(pattern_points, jai_points_2d_per_image, cv::Size(jai_width, jai_height), rvecs_jai, tvecs_jai);

	// extrinsic calibration from fz to jai
	extrinsicCalibration(fz_points_3d_vector, jai_points_2d_vector);

	// camera-robot calibration
	cameraRobotCalibration(rvecs_jai, tvecs_jai);

	// save calibration
	saveCalibration();
	calibrated_ = true;

	return success;
}

bool HandCameraCalibration::acquireCalibrationImages(int& jai_width, int& jai_height, int& fz_width, int& fz_height,
		std::vector<cv::Point2f>& jai_points_2d_vector, std::vector<cv::Point3f>& fz_points_3d_vector,
		std::vector< std::vector<cv::Point2f> >& jai_points_2d_per_image, std::vector< std::vector<cv::Point2f> >& fz_points_2d_per_image,
		const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images)
{
	int image_counter = 0;
	bool running = true;
	while (running == true)
	{
		int return_value = acquireCalibrationImage(jai_width, jai_height, fz_width, fz_height, jai_points_2d_vector, fz_points_3d_vector,
				jai_points_2d_per_image, fz_points_2d_per_image, pattern_size, black_white_threshold, load_images, num_images, image_counter);
		if (return_value > 0)
			break;
	}

	return true;
}

int HandCameraCalibration::acquireCalibrationImage(int& jai_width, int& jai_height, int& fz_width, int& fz_height,
		std::vector<cv::Point2f>& jai_points_2d_vector, std::vector<cv::Point3f>& fz_points_3d_vector,
		std::vector< std::vector<cv::Point2f> >& jai_points_2d_per_image, std::vector< std::vector<cv::Point2f> >& fz_points_2d_per_image,
		const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images, int& image_counter)
{
	int return_value = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	cv::Mat fz_image, fz_ir_image, jai_image;
	if (load_images == false)
	{
		// retrieve images from cameras
		bool success = true;
		success &= jai_go_5000_.grabFrame(jai_image);
		success &= fotonic_c70_.grabFrame(fz_cloud, fz_image, &fz_ir_image);
		cv::normalize(fz_ir_image, fz_ir_image, 0., 255., cv::NORM_MINMAX);
		cv::threshold(fz_ir_image, fz_ir_image, black_white_threshold, 255., cv::THRESH_BINARY);
		if (success == false)
			return -1;
	}
	else
	{
		// load files
		std::stringstream ss;
		ss << "telair/camera_calibration/" << image_counter;
		std::string jai_image_name = ss.str() + "_jai.png";
		jai_image = cv::imread(jai_image_name.c_str(), 0);
		std::string fz_ir_image_name = ss.str() + "_fz_ir.png";
		fz_ir_image = cv::imread(fz_ir_image_name.c_str(), 0);
		std::string fz_pcd_name = ss.str() + "_fz.pcd";
		pcl::io::loadPCDFile(fz_pcd_name, *fz_cloud);
		image_counter++;
		if (image_counter >= num_images)
			return_value = 1;
	}
	fz_width = fz_ir_image.cols;
	fz_height = fz_ir_image.rows;
	jai_width = jai_image.cols;
	jai_height = jai_image.rows;

	// find calibration pattern in fotonic ir image
	std::vector<cv::Point2f> fz_points_2d;
	bool pattern_found = cv::findChessboardCorners(fz_ir_image, pattern_size, fz_points_2d, cv::CALIB_CB_FILTER_QUADS);
	cv::Mat display_fz;
	cv::cvtColor(fz_ir_image, display_fz, CV_GRAY2BGR);
	cv::drawChessboardCorners(display_fz, pattern_size, cv::Mat(fz_points_2d), pattern_found);

	// find pattern in jai image
	std::vector<cv::Point2f> jai_points_2d;
	pattern_found = cv::findChessboardCorners(jai_image, pattern_size, jai_points_2d, cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_FILTER_QUADS);
	cv::Mat display_jai;
	cv::cvtColor(jai_image, display_jai, CV_GRAY2BGR);
	cv::drawChessboardCorners(display_jai, pattern_size, cv::Mat(jai_points_2d), pattern_found);

	// display
	cv::imshow("fz_ir_image", display_fz);
	cv::Mat display_jai_small;
	cv::resize(display_jai, display_jai_small, cv::Size(), 0.5, 0.5);
	cv::imshow("jai_image", display_jai_small);
	bool capture_image = false;
	char key = cv::waitKey(10);
	if (key == 'q')
		return_value = 1;
	else if (key == 'c')
		capture_image = true;
	else if (key!=-1)
	{
		if (key=='m')
			black_white_threshold += 1;
		else if (key=='n')
			black_white_threshold -= 1;
		std::cout << "black_white_threshold=" << black_white_threshold << std::endl;
	}

	// collect 3d-2d point correspondences
	if (fz_points_2d.size() == jai_points_2d.size() && jai_points_2d.size() == pattern_size.height*pattern_size.width)
	{
		// check whether this perspective was already captured
		bool already_captured = false;
		const double similarity_limit = 0.01*jai_width*jai_points_2d.size();
		for (size_t i=0; i<jai_points_2d_per_image.size(); ++i)
			if (cv::norm(cv::Mat(jai_points_2d), cv::Mat(jai_points_2d_per_image[i]), cv::NORM_L2) < similarity_limit)
			{
				already_captured = true;
				break;
			}
//			const double similarity_limit = 0.02*fz_width*fz_points_2d.size();
//			for (size_t i=0; i<fz_points_2d_per_image.size(); ++i)
//				if (cv::norm(cv::Mat(fz_points_2d), cv::Mat(fz_points_2d_per_image[i]), cv::NORM_L2) < similarity_limit)
//				{
//					already_captured = true;
//					break;
//				}

		// todo: clean hack
		if (/*already_captured == false || */capture_image == true || load_images == true)
		{
			// convert to 3d points
			for (size_t i=0; i<fz_points_2d.size(); ++i)
			{
				cv::Point3f mean_point = getMean3DCoordinate(fz_cloud, fz_points_2d[i].x, fz_points_2d[i].y);
				fz_points_3d_vector.push_back(mean_point);
//					pcl::PointXYZRGB& point = fz_cloud->at(fz_points_2d[i].x, fz_points_2d[i].y);
//					std::cout << "i=" << i << "  fz: " << cv::Point3f(point.x, point.y, point.z) << std::endl;
//					fz_points_3d.push_back(cv::Point3f(point.x, point.y, point.z));
			}
			// collect 2d points
			fz_points_2d_per_image.push_back(fz_points_2d);
			jai_points_2d_vector.insert(jai_points_2d_vector.end(), jai_points_2d.begin(), jai_points_2d.end());
			jai_points_2d_per_image.push_back(jai_points_2d);

			// save images
			if (load_images == false)
			{
				std::stringstream ss;
				ss << "telair/camera_calibration/" << image_counter;
				std::string jai_image_name = ss.str() + "_jai.png";
				cv::imwrite(jai_image_name.c_str(), jai_image);
				std::string fz_ir_image_name = ss.str() + "_fz_ir.png";
				cv::imwrite(fz_ir_image_name.c_str(), fz_ir_image);
				std::string fz_pcd_name = ss.str() + "_fz.pcd";
				pcl::io::savePCDFile(fz_pcd_name, *fz_cloud, false);
				++image_counter;
			}
		}
	}
	else
		return_value = -2;
	std::cout << "Captured perspectives: " << fz_points_2d_per_image.size() << std::endl;

	return return_value;
}

void HandCameraCalibration::computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const int number_images)
{
	// prepare chessboard 3d points
	pattern_points.clear();
	pattern_points.resize(1);
	pattern_points[0].resize(pattern_size.height*pattern_size.width);
	const double chessboard_width = 0.05;	// chessboard cell width in [m]
	for (int v=0; v<pattern_size.height; ++v)
		for (int u=0; u<pattern_size.width; ++u)
			pattern_points[0][v*pattern_size.width+u] = cv::Point3f(u*chessboard_width, v*chessboard_width, 0.f);
	pattern_points.resize(number_images, pattern_points[0]);
}

void HandCameraCalibration::intrinsicCalibrationJai(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs_jai, std::vector<cv::Mat>& tvecs_jai)
{
	std::cout << "Intrinsic calibration JAI go 5000 started ..." << std::endl;
	K_jai_ = cv::Mat::eye(3, 3, CV_64F);
	distortion_jai_ = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(pattern_points, camera_points_2d_per_image, image_size, K_jai_, distortion_jai_, rvecs_jai, tvecs_jai);
	std::cout << "Intrinsic calibration JAI go 5000:\nK_jai:\n" << K_jai_ << "\ndistortion_jai:\n" << distortion_jai_ << std::endl;
}

void HandCameraCalibration::extrinsicCalibration(const std::vector<cv::Point3f>& fz_points_3d_vector, const std::vector<cv::Point2f>& jai_points_2d_vector)
{
	// clean list of 3d points from invalid measurements
	std::vector<cv::Point3f> fz_points_3d_cleaned;
	std::vector<cv::Point2f> jai_points_2d_cleaned;
	for (size_t i=0; i<fz_points_3d_vector.size(); ++i)
	{
		if (!(fz_points_3d_vector[i].x==0.f && fz_points_3d_vector[i].y==0.f && fz_points_3d_vector[i].z==0.f) || fz_points_3d_vector[i].z!=fz_points_3d_vector[i].z)
		{
			fz_points_3d_cleaned.push_back(fz_points_3d_vector[i]);
			jai_points_2d_cleaned.push_back(jai_points_2d_vector[i]);
		}
	}

	// extrinsic calibration from fz to jai
	cv::Mat rvec;
	cv::solvePnP(fz_points_3d_cleaned, jai_points_2d_cleaned, K_jai_, distortion_jai_, rvec, t_, false, cv::ITERATIVE);
	cv::Rodrigues(rvec, R_);
	std::cout << "solvePnP: Extrinsinc calibration:\nR:\n" << R_ << "\nt:\n" << t_ << std::endl;
}

void HandCameraCalibration::cameraRobotCalibration(const std::vector<cv::Mat>& rvecs_jai, const std::vector<cv::Mat>& tvecs_jai)
{
	// arm-camera calibration
	// x, y, z, w, p, r (= yaw, pitch, roll with 1. rotation = roll around z, 2. rotation = pitch around y', 3. rotation around x'')
	cv::Mat arm_positions = (cv::Mat_<float>(39,6) <<
			32.534,	1391.635,	436.573,	179.833,	-0.005,	-119.501,
			2.678, 	1391.616, 	436.656, 	179.830, 	-0.002, 	-119.500,
			-30.383, 	1391.613, 	436.734, 	179.828, 	0.0, 	-115.375,
			-30.354, 	1416.441, 	436.826, 	179.824, 	0.0, 	-123.394,
			14.688, 	1409.591, 	436.908, 	179.821, 	0.003, 	-123.393,
			47.042, 	1409.565, 	436.979, 	179.819, 	0.005, 	-123.392,
			32.214, 	1324.595, 	437.057, 	179.816, 	0.008, 	-123.392,
			0.259, 	1324.575, 	437.117, 	179.813, 	0.010, 	-123.393,
			-18.412, 	1324.571, 	437.180, 	179.811, 	0.011, 	-126.529,
			-18.558, 	1361.942, 	437.416, 	-174.848, 	-7.158, 	-126.864,
			23.253, 	1408.345, 	437.497, 	-179.405, 	-10.554, 	-126.740,
			-10.261, 	1423.790, 	437.589, 	-169.205, 	-2.724, 	-120.907,
			-14.834, 	1353.210, 	437.828, 	-175.886, 	8.687, 	-114.118,
			-14.842, 	1353.177, 	724.456, 	-179.104, 	-1.929, 	-120.782,
			-153.997, 	1374.504, 	724.623, 	-179.114, 	-1.930, 	-120.780,
			189.747, 	1360.784, 	724.693, 	-179.122, 	-1.932, 	-120.780,
			189.769, 	1507.976, 	724.787, 	-179.128, 	-1.931, 	-120.780,
			-0.612, 	1507.938, 	724.906, 	-179.131, 	-1.927, 	-120.780,
			-165.797, 	1507.902, 	725.009, 	-179.137, 	-1.924, 	-120.782,
			-165.744, 	1196.436, 	725.050, 	-179.138, 	-1.923, 	-120.782,
			18.52, 	1196.407, 	725.085, 	-179.144, 	-1.926, 	-120.782,
			188.480, 	1196.395, 	725.118, 	178.739, 	1.616, 	-120.779,
			141.716, 	1253.977, 	725.170, 	-176.572, 	-6.232, 	-117.108,
			112.132, 	1440.772, 	725.236, 	-177.507, 	-8.927, 	-127.909,
			-123.648, 	1440.739, 	725.330, 	-168.315, 	-1.664, 	-128.759,
			-123.585, 	1254.576, 	725.438, 	-171.849, 	4.523, 	-114.970,
			-236.631, 	1195.501, 	878.487, 	-177.723, 	0.000, 	-120.557,
			-14.921, 	1191.495, 	878.542, 	-177.731, 	-0.002, 	-120.556,
			198.597, 	1191.471, 	849.785, 	174.825, 	1.025, 	-120.864,
			198.590, 	1337.231, 	849.844, 	174.818, 	1.024, 	-120.864,
			-7.679, 	1337.192, 	849.915, 	174.810, 	1.024, 	-120.862,
			-201.665, 	1337.163, 	849.988, 	178.435, 	3.185, 	-120.731,
			-201.657, 	1501.781, 	850.069, 	178.432, 	3.187, 	-120.731,
			21.280, 	1501.731, 	850.156, 	178.429, 	3.192, 	-120.728,
			172.489, 	1501.685, 	850.231, 	178.423, 	3.193, 	-120.727,
			188.268, 	1501.638, 	850.321, 	167.294, 	-3.714, 	-115.901,
			-188.132, 	1501.535, 	850.490, 	-171.924, 	6.441, 	-112.471,
			-188.130, 	1302.978, 	850.634, 	-172.230, 	6.821, 	-124.221,
			141.221, 	1253.279, 	850.679, 	174.824, 	-2.199, 	-129.705);

	Eigen::Quaterniond avg_rotation(0,0,0,0);
	Eigen::Vector3d avg_translation(0,0,0);
	for (size_t index=0; index<rvecs_jai.size(); ++index)
	{
		std::cout << "\n-----" << index << "-----\n";
		// todo: remove if TCP is defined and TCP coordinates are provided
		// transform J6 to TCP
		// todo: dynamically obtain TCP
		double j6_to_tcp_roll = -32./180. * CV_PI - 90./180. * CV_PI;
		double j6_to_tcp_pitch = 180./180. * CV_PI;
		double j6_to_tcp_z = 0.2671;		// checkerboard height not included
		cv::Mat T_j6_to_tcp = makeTransform(rotationMatrixFromRPY(j6_to_tcp_roll, j6_to_tcp_pitch, 0), cv::Mat(cv::Vec3d(0, 0, j6_to_tcp_z)));
		//std::cout << "\n T_j6_to_tcp:\n" << T_j6_to_tcp << std::endl;

		// transform world to j6
		cv::Mat T_world_to_j6 = makeTransform(rotationMatrixFromRPY(arm_positions.at<float>(index,5)/180.*CV_PI, arm_positions.at<float>(index,4)/180.*CV_PI, arm_positions.at<float>(index,3)/180.*CV_PI), cv::Mat(cv::Vec3d(0.001*arm_positions.at<float>(index,0), 0.001*arm_positions.at<float>(index,1), 0.001*arm_positions.at<float>(index,2))));
		//std::cout << "\n T_world_to_j6:\n" << T_world_to_j6 << std::endl;

		// transform world to tcp
		cv::Mat T_world_to_tcp = T_world_to_j6 * T_j6_to_tcp;
		//std::cout << "\n T_world_to_tcp:\n" << T_world_to_tcp << std::endl;

		// transform tcp to checkerboard
		cv::Mat T_tcp_to_checkerboard = makeTransform(cv::Mat::eye(3,3,CV_64FC1), cv::Mat(cv::Vec3d(-0.125, -0.075, -0.006)));

		cv::Mat T_world_to_checkerboard = T_world_to_tcp * T_tcp_to_checkerboard;
		//std::cout << "\n T_world_to_checkerboard:\n" << T_world_to_checkerboard << std::endl;

		// transform checkerboard to jai camera per rvecs_jai, tvecs_jai,
		cv::Mat R, t;
		cv::Rodrigues(rvecs_jai[index], R);
		t = -R.t()*tvecs_jai[index];
		cv::Mat T_checkerboard_to_jai = makeTransform(R.t(), t);
		//std::cout << "\n T_checkerboard_to_jai:\n" << T_checkerboard_to_jai << std::endl;

		// transform jai camera to fz camera
		cv::Mat T_jai_to_fz = makeTransform(R_, t_);
		//std::cout << "\n T_jai_to_fz:\n" << T_jai_to_fz << std::endl;

		cv::Mat T_world_fz = T_world_to_checkerboard*T_checkerboard_to_jai*T_jai_to_fz;
		std::cout << "\n T_world_fz:\n" << T_world_fz << std::endl;

		Eigen::Matrix3d m;
		for (int i=0;i<3;++i)
			for (int j=0;j<3;++j)
				m(i,j) = T_world_fz.at<double>(i,j);
		Eigen::Quaterniond q(m);
		avg_rotation.w() += q.w();
		avg_rotation.x() += q.x();
		avg_rotation.y() += q.y();
		avg_rotation.z() += q.z();
		for (int i=0; i<3; ++i)
			avg_translation(i) += T_world_fz.at<double>(i,3);
	}
	float number = rvecs_jai.size();
	avg_rotation.w() /= number;
	avg_rotation.x() /= number;
	avg_rotation.y() /= number;
	avg_rotation.z() /= number;
	Eigen::Matrix3d m = avg_rotation.matrix();
	R_world_to_fz_.create(3,3,CV_64FC1);
	for (int i=0;i<3;++i)
		for (int j=0;j<3;++j)
			R_world_to_fz_.at<double>(i,j) = m(i,j);
	avg_translation /= number;
	t_world_to_fz_.create(3,1,CV_64FC1);
	for (int i=0; i<3; ++i)
		t_world_to_fz_.at<double>(i,0) = avg_translation(i);

	std::cout << "\n-------------------------\nR_world_to_fz:\n" << R_world_to_fz_ << "\nt_world_to_fz:\n" << t_world_to_fz_ << std::endl;
}

bool HandCameraCalibration::testCalibration()
{
	const cv::Size pattern_size(6,4);
	int black_white_threshold = 43;

	// setup cameras
	bool success = true;
	//success &= openCameras();
	success &= fotonic_c70_.setParametersCalibration(1, 40);
	jai_go_5000_.setFeatureFloat("AcquisitionFrameRate", 40.0);
	//success &= startCameras();
	if (success == false)
		return success;

	// test calibration
	bool running = true;
	while (running == true)
	{
		// retrieve images from cameras
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr fz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		cv::Mat fz_image, fz_ir_image, jai_image;
		success &= jai_go_5000_.grabFrame(jai_image);
		success &= fotonic_c70_.grabFrame(fz_cloud, fz_image, &fz_ir_image);
		cv::normalize(fz_ir_image, fz_ir_image, 0., 255., cv::NORM_MINMAX);
		cv::threshold(fz_ir_image, fz_ir_image, black_white_threshold, 255., cv::THRESH_BINARY);

		// find calibration pattern 3d points from fz sensor
		std::vector<cv::Point2f> fz_points_2d;
		bool pattern_found = cv::findChessboardCorners(fz_ir_image, pattern_size, fz_points_2d, cv::CALIB_CB_FILTER_QUADS);
		cv::Mat display_fz;
		cv::cvtColor(fz_ir_image, display_fz, CV_GRAY2BGR);
		cv::drawChessboardCorners(display_fz, pattern_size, cv::Mat(fz_points_2d), pattern_found);
		cv::Mat display_jai, temp;
		cv::undistort(jai_image, temp, K_jai_, distortion_jai_);
		cv::cvtColor(temp, display_jai, CV_GRAY2BGR);

		// convert all xyz coordinates of fz sensor to jai image coordinates
		if (pattern_found == true)
		{
			std::vector< cv::Point3f > fz_points_3d_test(fz_points_2d.size());
			for (size_t i=0; i<fz_points_2d.size(); ++i)
			{
				fz_points_3d_test[i] = getMean3DCoordinate(fz_cloud, fz_points_2d[i].x, fz_points_2d[i].y);
				cv::Point3d mean_point = cv::Point3d(fz_points_3d_test[i].x, fz_points_3d_test[i].y, fz_points_3d_test[i].z);
//				pcl::PointXYZRGB& point = fz_cloud->at(fz_points_2d[i].x, fz_points_2d[i].y);
//				fz_points_3d_test[i] = cv::Point3f(point.x, point.y, point.z);
				cv::Mat point_jai_mat = K_jai_ * (R_ * cv::Mat(mean_point) + t_);
				cv::circle(display_jai, cv::Point(point_jai_mat.at<double>(0)/point_jai_mat.at<double>(2), point_jai_mat.at<double>(1)/point_jai_mat.at<double>(2)), 4, CV_RGB(0,255,0), 2);
			}
			cv::Mat rvec;
			cv::Rodrigues(R_, rvec);
			std::vector< cv::Point2f> jai_points_projected;
			cv::projectPoints(fz_points_3d_test, rvec, t_, K_jai_, distortion_jai_, jai_points_projected);
			for (size_t i=0; i<jai_points_projected.size(); ++i)
				cv::circle(display_jai, cv::Point(jai_points_projected[i].x, jai_points_projected[i].y), 4, CV_RGB(0,0,255), 2);
		}

		// display
		cv::imshow("fz_ir_image", display_fz);
		cv::imshow("jai_image", display_jai);
		char key = cv::waitKey(10);
		if (key == 'q')
			running = false;
	}

	// clean up
	//stopCameras();
	//closeCameras();

	return success;
}

bool HandCameraCalibration::saveCalibration()
{
	bool success = true;

	// save calibration
	cv::FileStorage fs("telair/camera_calibration/camera_calibration.yml", cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "K_jai" << K_jai_;
		fs << "distortion_jai" << distortion_jai_;
		fs << "R" << R_;
		fs << "t" << t_;
		fs << "R_world_to_fz" << R_world_to_fz_;
		fs << "t_world_to_fz" << t_world_to_fz_;
	}
	else
	{
		std::cout << "Error: HandCameraCalibration::calibrateCameras: Could not write calibration to file.";
		success = false;
	}
	fs.release();

	return success;
}

bool HandCameraCalibration::loadCalibration()
{
	bool success = true;

	// load calibration
	cv::FileStorage fs("telair/camera_calibration/camera_calibration.yml", cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["K_jai"] >> K_jai_;
		fs["distortion_jai"] >> distortion_jai_;
		fs["R"] >> R_;
		fs["t"] >> t_;
		fs["R_world_to_fz"] >> R_world_to_fz_;
		fs["t_world_to_fz"] >> t_world_to_fz_;
	}
	else
	{
		std::cout << "Error: HandCameraCalibration::calibrateCameras: Could not read calibration from file.";
		success = false;
	}
	fs.release();

	calibrated_ = true;

	return success;
}

void HandCameraCalibration::getCalibration(cv::Mat& K_jai, cv::Mat& distortion_jai, cv::Mat& R, cv::Mat& t)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	K_jai = K_jai_.clone();
	distortion_jai = distortion_jai_.clone();
	R = R_.clone();
	t = t_.clone();
}

void HandCameraCalibration::getCameraWorldPose(cv::Mat& R_world_to_fz, cv::Mat& t_world_to_fz)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	R_world_to_fz = R_world_to_fz_.clone();
	t_world_to_fz = t_world_to_fz_.clone();
}

void HandCameraCalibration::projectFz3dToJai2d(const std::vector<cv::Point3f>& fz_points_3d, std::vector<cv::Point2f>& jai_points_projected)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::Mat rvec;
	cv::Rodrigues(R_, rvec);
	cv::projectPoints(fz_points_3d, rvec, t_, K_jai_, distortion_jai_, jai_points_projected);
}

void HandCameraCalibration::undistortJai(const cv::Mat& jai_image, cv::Mat& jai_image_undistorted)
{
	if (calibrated_ == false && loadCalibration() == false)
	{
		std::cout << "Error: HandCameraCalibration not calibrated and no calibration data available on disk." << std::endl;
		return;
	}

	cv::undistort(jai_image, jai_image_undistorted, K_jai_, distortion_jai_);
}

cv::Point3f HandCameraCalibration::getMean3DCoordinate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int x, const int y)
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
