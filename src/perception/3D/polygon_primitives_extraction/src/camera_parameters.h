/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
 * @addtogroup camera_parameters 
 * @{
 * @file
 * @brief Defines a class where the parameters of a camera are stored
 */
#ifndef _CAMERA_PARAMETERS_H_
#define _CAMERA_PARAMETERS_H_


//####################################################################
//// Includes:
////####################################################################
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>

//Define the class
class class_camera_parameters
{
	// ------------------------------------------------------------------------------
	//PUBLIC SECTION OF THE CLASS
	// ------------------------------------------------------------------------------
	public:

		// Constructor. 
		class_camera_parameters(void)
		{
			intrinsic.create(3, 4, CV_64FC1); //allocate intrinsic Matrix
			extrinsic.create(4, 4, CV_64FC1); //allocate extrinsic Matrix
			extrinsic_inverse.create(4, 4, CV_64FC1); //allocate the inverted extrinsic Matrix
			distortion.create(1, 5, CV_64FC1); //allocate the inverted extrinsic Matrix
		};

		
		~class_camera_parameters(void)
		{
		
		};

		int set_intrinsic(double fx, double fy, double cx, double cy);
		int set_extrinsic(double p11, double p12, double p13, double p14,
			              double p21, double p22, double p23, double p24,
    				      double p31, double p32, double p33, double p34);
		int set_extrinsic(tf::StampedTransform* t);
		int set_extrinsic(tf::Transform* t);

		int set_distortion(double k1, double k2, double p1, double p2, double k3)
		{
			cv::Mat *M = &distortion;
			M->at<double>(0,0) = k1; M->at<double>(0,1) = k2; M->at<double>(0,2) = p1; M->at<double>(0,3) = p2; M->at<double>(0,4) = k3;
			return 1;
		}

		int set_distortion_spherical(double calibration_width, double calibration_height, double cx, double cy, double param, double image_scale=1)
		{
			spherical_distortion_params.cx=cx;
			spherical_distortion_params.cy=cy;
			spherical_distortion_params.param=param;
			spherical_distortion_params.scale=image_scale;
			spherical_distortion_params.calibration_width=calibration_width;
			spherical_distortion_params.calibration_height=calibration_height;
			return 1;
		}

		int set_width_height_num_channels(int w, int h, int nc)
		{
			width = w;
		    height = h;
			num_channels = nc;	
			return 1;
		}

	cv::Mat distortion; //the distortion matrix 1,5 cv_64F
	cv::Mat intrinsic;  //the intrinsic matrix 3,4 cv_64F
	cv::Mat extrinsic;  //the extrinsic matrix 4,4 cv_64F
	cv::Mat extrinsic_inverse;  //the extrinsic matrix 4,4 cv_64F
	cv::Mat transformation;  //the extrinsic matrix 4,4 cv_64F
	std::vector<pcl::PointXY> pixels_canvas; //a list of points correcponding to the canvas
	ros::Time stamp;
	int width;
	int height;
	int num_channels;
	tf::StampedTransform camera_6dof_position;

	struct
	{
	double calibration_width;
	double calibration_height;
	double cx,cy,param;
	double scale;
	}spherical_distortion_params;
};

#endif
/**
 *@}
 */      
