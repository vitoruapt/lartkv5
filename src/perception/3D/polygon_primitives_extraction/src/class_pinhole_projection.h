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
 * @addtogroup pinhole_projection
 * @{
 * @file
 * @brief header for pinhole projection. Defines default distortions parameters
 * for mit dataset cameras.
 */

#ifndef _CLASS_PINHOLE_PROJECTION_H_
#define _CLASS_PINHOLE_PROJECTION_H_

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/tf.h>

#include "camera_parameters.h"

#ifndef PFLN
#define PFLN {printf("File %s Line %d\n",__FILE__, __LINE__);}
#endif
class class_pinhole_projection: virtual public class_camera_parameters
{
	public:

		class_pinhole_projection(void)
		{
		}

		~class_pinhole_projection(void)
		{
		}

		///Sets and gets
		int set_projection_plane(double a, double b, double c, double d);

		
		///Projections
		int project_vertex_to_pixel(const pcl::PointCloud<pcl::PointXYZ>::Ptr v, std::vector<pcl::PointXY>* p);
		int project_pixel_to_vertex(const std::vector<pcl::PointXY>* p, pcl::PointCloud<pcl::PointXYZ>* v);

		int project_pixel_with_color_to_vertex(const pcl::PointCloud<pcl::PointXYZRGB>* p, pcl::PointCloud<pcl::PointXYZRGB>* v);


		///Computation
		int compute_transformation_matrix(void);

		///Prints
		int print_CvMat(cv::Mat *mat, const char* name);


struct 
{
	double a,b,c,d;
}projection_plane;


	private:


};



#endif
/**
 *@}
 */      
