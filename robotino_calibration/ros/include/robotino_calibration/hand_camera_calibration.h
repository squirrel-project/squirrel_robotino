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

#ifndef __HAND_CAMERA_CALIBRATION_H__
#define __HAND_CAMERA_CALIBRATION_H__

// OpenCV
#include <opencv/cv.h>

// PCL
#include <pcl/point_cloud.h>


//// ROS
//#include <ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>

// compute rotation matrix from roll, pitch, yaw
// (w, p, r) = (yaW, Pitch, Roll) with
// 1. rotation = roll around z
// 2. rotation = pitch around y'
// 3. rotation = yaw around x''
cv::Mat rotationMatrixFromRPY(double roll, double pitch, double yaw);

cv::Mat makeTransform(const cv::Mat& R, const cv::Mat& t);


class HandCameraCalibration
{
public:

	HandCameraCalibration();

	~HandCameraCalibration();

	bool calibrateHandToCameraExtrinsicOnly(const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images);

	// runs cameras in calibration mode and publishes respective images such that an external calibration software can do the calibration
	bool calibrateHandToCamera(const cv::Size pattern_size = cv::Size(6,4), int black_white_threshold = 43, const bool load_images = false, const int num_images = 94);

	// acquires images manually until user interrupts
	bool acquireCalibrationImages(int& jai_width, int& jai_height, int& fz_width, int& fz_height,
			std::vector<cv::Point2f>& jai_points_2d_vector, std::vector<cv::Point3f>& fz_points_3d_vector,
			std::vector< std::vector<cv::Point2f> >& jai_points_2d_per_image, std::vector< std::vector<cv::Point2f> >& fz_points_2d_per_image,
			const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images);

	// acquire a single image, can be used within automatic image capture
	int acquireCalibrationImage(int& jai_width, int& jai_height, int& fz_width, int& fz_height,
			std::vector<cv::Point2f>& jai_points_2d_vector, std::vector<cv::Point3f>& fz_points_3d_vector,
			std::vector< std::vector<cv::Point2f> >& jai_points_2d_per_image, std::vector< std::vector<cv::Point2f> >& fz_points_2d_per_image,
			const cv::Size pattern_size, int black_white_threshold, const bool load_images, const int num_images, int& image_counter);

	void computeCheckerboard3dPoints(std::vector< std::vector<cv::Point3f> >& pattern_points, const cv::Size pattern_size, const int number_images);

	void intrinsicCalibrationJai(const std::vector< std::vector<cv::Point3f> >& pattern_points, const std::vector< std::vector<cv::Point2f> >& camera_points_2d_per_image, const cv::Size& image_size, std::vector<cv::Mat>& rvecs_jai, std::vector<cv::Mat>& tvecs_jai);

	void extrinsicCalibration(const std::vector<cv::Point3f>& fz_points_3d_vector, const std::vector<cv::Point2f>& jai_points_2d_vector);

	void cameraRobotCalibration(const std::vector<cv::Mat>& rvecs_jai, const std::vector<cv::Mat>& tvecs_jai);

	void setCalibrationStatus(bool calibrated)
	{
		calibrated_ = calibrated;
	}

	// find checkerboard corners in fotonic ir image and projects them into the jai image
	bool testCalibration();

	bool saveCalibration();
	bool loadCalibration();

	void getCalibration(cv::Mat& K_jai, cv::Mat& distortion_jai, cv::Mat& R, cv::Mat& t);

	void getCameraWorldPose(cv::Mat& R_world_to_fz, cv::Mat& t_world_to_fz);

	void projectFz3dToJai2d(const std::vector<cv::Point3f>& fz_points_3d, std::vector<cv::Point2f>& jai_points_projected);

	void undistortJai(const cv::Mat& jai_image, cv::Mat& jai_image_undistorted);

protected:

	cv::Mat K_jai_;			// intrinsic matrix for JAI camera
	cv::Mat distortion_jai_;	// distortion parameters for JAI camera

	cv::Mat R_;		// extrinsic calibration: rotation matrix from fz_sensor to jai sensor
	cv::Mat t_;		// extrinsic calibration: translation vector from fz_sensor to jai sensor

	cv::Mat R_world_to_fz_;	// robot-camera calibration: rotation matrix from world to fz_sensor
	cv::Mat t_world_to_fz_;	// robot-camera calibration: translation vector from world to fz_sensor

	bool calibrated_;	// only true if cameras were calibrated or a calibration was loaded before

	cv::Point3f getMean3DCoordinate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const int x, const int y);
};

#endif // __HAND_CAMERA_CALIBRATION_H__
