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
 * @addtogroup xb3
 * @file
 * @brief Main header file for the xb3 module
 *@{
 */

#ifndef _XB3_H_
#define _XB3_H_

//####################################################################
// Includes:
//####################################################################

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>
//#include <driver_base/driver.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>




#include <signal.h>
//=============================================================================
// PGR Includes
//=============================================================================
#include "pgr_registers.h"
#include "pgr_stereocam.h"

#include <xb3/xb3Config.h>
#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}



#ifdef _XB3_CPP_

/** \struct tag_t_flags
 *  \brief set debug flag
 */
struct tag_t_flags{
	char debug;
}t_flags;

/** \struct tag_t_buffers
 *  \brief Image buffers for libpgrlibdcstereo
 */
struct tag_t_buffers{
	unsigned int nBufferSize;
	unsigned char* pucRightRGB;
	unsigned char* pucLeftRGB;
	unsigned char* pucCenterRGB;
	unsigned char* pucGreenBuffer;
	unsigned char* pucRGBBuffer;
	unsigned char* pucDeInterlacedBuffer;
}t_buffers;

/** \struct tag_t_dc1394
 *  \brief Camera register for libpgrlibdcstereo
 */
struct tag_t_dc1394{
	PGRStereoCamera_t stereoCamera;
	unsigned int nThisCam;
	dc1394_t * d;
	dc1394camera_t* 	camera;
}t_dc1394;

/** \struct tag_t_imgs
 *  \brief cv::Mat Image buffers.
 */
struct tag_t_imgs{
	cv::Mat left;
	cv::Mat center;
	cv::Mat right;
	cv::Mat left_640_480;
	cv::Mat right_640_480;
	cv::Mat center_640_480;
}t_imgs;

/** \struct tag_t_msgs
 *  \brief ROS sensor_msgs with camera info manager.
 */
struct tag_t_msgs{
	sensor_msgs::Image short_left;                                                                                                                                                                   
	sensor_msgs::Image short_right;
	sensor_msgs::Image wide_left;
	sensor_msgs::Image wide_right;

	sensor_msgs::CameraInfo   short_left_info;
	sensor_msgs::CameraInfo   short_right_info;
	sensor_msgs::CameraInfo   wide_left_info;
	sensor_msgs::CameraInfo   wide_right_info;

	image_transport::CameraPublisher short_left_pub; 
	image_transport::CameraPublisher short_right_pub;
	image_transport::CameraPublisher wide_left_pub; 
	image_transport::CameraPublisher wide_right_pub;


	camera_info_manager::CameraInfoManager *short_left_info_manager;
	camera_info_manager::CameraInfoManager *short_right_info_manager;
	camera_info_manager::CameraInfoManager *wide_left_info_manager;
	camera_info_manager::CameraInfoManager *wide_right_info_manager;

	std::string short_left_info_url;
	std::string short_right_info_url;
	std::string wide_left_info_url;
	std::string wide_right_info_url;

}t_msgs;


#endif

#endif
/**
*@}
*/
