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
 * @addtogroup colormap
 * @file 
 * @brief header file for colormap class
 *@{
 */
#ifndef _colormap_H_
#define _colormap_H_

//####################################################################
// Includes:
//####################################################################
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>      

#include <opencv2/core/core.hpp>


class class_colormap
{
	public:
		class_colormap(std::string name, int total, float alfa, bool reverse=false);
		~class_colormap(void);

		std_msgs::ColorRGBA color(int i);
		cv::Scalar cv_color(int i);
		
	private:
		std::vector<std_msgs::ColorRGBA> cm;
		int max_index;



	int setup_colormap(int total, float alfa, bool reverse, float* r, float* g, float* b);

	int init_colormap_jet(int total, float alfa, bool reverse);
	int init_colormap_hsv(int total, float alfa, bool reverse);
	int init_colormap_hot(int total, float alfa, bool reverse);
	int init_colormap_cool(int total, float alfa, bool reverse);
	int init_colormap_spring(int total, float alfa, bool reverse);
	int init_colormap_summer(int total, float alfa, bool reverse);
	int init_colormap_autumn(int total, float alfa, bool reverse);
	int init_colormap_winter(int total, float alfa, bool reverse);
	int init_colormap_gray(int total, float alfa, bool reverse);
	int init_colormap_bone(int total, float alfa, bool reverse);
	int init_colormap_copper(int total, float alfa, bool reverse);
	int init_colormap_pink(int total, float alfa, bool reverse);
	int init_colormap_lines(int total, float alfa, bool reverse);
};

#endif
/**
 *@}
*/
