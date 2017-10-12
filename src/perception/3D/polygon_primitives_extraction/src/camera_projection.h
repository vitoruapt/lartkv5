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
 * @addtogroup camera_projection 
 * @{
 * @file 
 * @brief The class camera_projection performs the projection of an image into a polygon plane
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-09-15
 */
#ifndef _CAMERA_PROJECTION_H_
#define _CAMERA_PROJECTION_H_

#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}

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

//include
#include "class_pinhole_projection.h"
#include "polygon_intersection.h"
#include <bo_polygon2d/polygon_boolean_operations.h>
#include "distortion_correction.h"
#include "projection_mesh.h"

typedef cv::Vec<unsigned char, 3> t_pixel;

/**
 * @brief The class camera projection. Provides functionalities to perform projection of an image into a polygon plane
 */
class class_camera_projection: public class_pinhole_projection, public class_distortion_correction, public class_polygon_boolean_operations, virtual public class_camera_parameters, public class_projection_mesh 
{
	// ------------------------------------------------------------------------------
	//PUBLIC SECTION OF THE CLASS
	// ------------------------------------------------------------------------------
	public:

		/**
		 * @brief Constructor. 
		 */
		class_camera_projection(void){is_mapped_to_mesh=false; projection_name="unknown";};

		/**
		 * @brief The destrictor. Does nothing.
		 */
		~class_camera_projection(void){};

		/**
		 * @brief Sets the image to be projected. Also computes the canvas, canny and grid masks
		 *
		 * @param image the image to be projected
		 * @param time the time stamp of the image
		 * @param name the name of the camera from which the image is taken
		 *
		 * @return 1=success, 0=failure
		 */
		int set_image(const cv::Mat* image, ros::Time time, std::string name="none");

		/**
		 * @brief Creates the image canvas pixels list and stores it to 
		 *
		 * @param image
		 * @param pts
		 *
		 * @return 
		 */
		int create_image_canvas(const cv::Mat* image, std::vector<pcl::PointXY>* pts);
		int create_image_canvas_for_known_camera(const cv::Mat* image, std::vector<pcl::PointXY>* pts, std::string name);
		int create_polygon_from_pts_list(const pcl::PointCloud<pcl::PointXYZ>* vertexes, pcl::PointCloud<pcl::PointXYZ>* vertices);
		float get_weight_for_pixel(int l, int c, double resolution_factor, float px, float py, float pz);

		int set_vertex_chull(const pcl::PointCloud<pcl::PointXYZ>::Ptr vertexes);	

		void compute_projectable_pixels(void);
		int compute_pixel_list_to_vertex_list(pcl::ModelCoefficientsPtr coeff);

		//Draw
		int draw_pixels_vector(cv::Mat* image, std::vector<pcl::PointXY>* pts, const cv::Scalar color, int thickness=1);
		int draw_pixels_projectable(cv::Mat* image, pcl::PointCloud<pcl::PointXYZRGB>* pc, const cv::Scalar color, int thickness=1);
		int draw_pixels_vector_as_polyline(cv::Mat* image, std::vector<pcl::PointXY>* pts, const cv::Scalar color, int thickness);

		
		std::string projection_name;
		cv::Mat image_original;
		cv::Mat image_corrected;
		cv::Mat image_gui;
		//cv::Mat image_hough;
		//cv::Mat mask_hough;
		cv::Mat image_gray;
		cv::Mat mask_canny;
		cv::Mat mask_grid;
		cv::Mat mask_intersection;
		cv::Mat mask_projectable_pixels;
		//cv::Mat mask_tmp;

		pcl::PointCloud<pcl::PointXYZ> vertex_canvas;
		pcl::PointCloud<pcl::PointXYZ> vertex_chull;
		pcl::PointCloud<pcl::PointXYZ> vertex_intersection;
		std::vector<pcl::PointXY> pixels_intersection;
		pcl::PointCloud<pcl::PointXYZRGB> pixels_projectable;
		pcl::PointCloud<pcl::PointXYZRGB> vertex_projectable;
		//std::vector<float> vertex_projectable_weight;
		//cv::vector<cv::Vec4i> lines;


		pcl::PointCloud<pcl::PointXYZRGB> pixel_list;
		pcl::PointCloud<pcl::PointXYZRGB> vertex_list;
		std::vector<float> face_weight;

		bool is_mapped_to_mesh;
		// ------------------------------------------------------------------------------
		//PRIVATE SECTION OF THE CLASS
		// ------------------------------------------------------------------------------

	private:
};

#endif
/**
 *@}
 */      
