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
#ifndef _plane_model_segmentation_H_
#define _plane_model_segmentation_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// ROS Transform stuff 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// Point cloud library stuff filters and so on

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define pi 3.1415926

/**
 * \file
 \brief A class to determine a planar model for road detection
 \author Tiago Talhada
 **/


/**
 * \class plane_model_road
 * 
 * This is a class to detect the road reference frame
 * \date March 2012
 * \author ttalhada
 * 
 */

template <class T>
class plane_model_road
{
public:
  
  /**
   * \brief get the road reference frame
   * 
   T h*is function does all the work for you. here are the main steps:
   STEP 1 create the model coeeficients
   STEP 2 project origin to plane, for example
   STEP 3 Find the difference between the cloud frame_id to ground
   STEP 4 Calculate the difference  z axis
   * \param[in] cloud_in
   * \return bool success
   */
  bool find_road_frame(pcl::PointCloud<T> cloud_in);
  
  /**
   * \brief overload function for ros msgs
   * Converts sensor_msgs::PointCloud2 to pcl and calls the function find_road_frame(pcl::PointCloud< T> pc)
   * \param[in] msg
   * \return success
   */
  bool find_road_frame(const sensor_msgs::PointCloud2ConstPtr& msg);
  
  /**
   * \brief get and publish the road frame
   T his f*unction does all the work for you. here are the main steps:
   STEP 1 create the model coeeficients
   STEP 2 project origin to plane, for example
   STEP 3 Find the difference between the cloud frame_id to ground
   STEP 4 Calculate the difference  z axis
   STEP 5 Publish the road frame
   * \param[in] origin_projected
   * \return tf::Transform
   */
  tf::Transform find_and_publish_road_frame(pcl::PointCloud<T> cloud_in);
  
  /**
   * \brief overload function for ros msgs
   * Converts sensor_msgs::PointCloud2 to pcl and calls the function find_and_publish_road_frame(pcl::PointCloud< T> pc)
   * \param[in] msg
   * \return transform
   */
  tf::Transform find_and_publish_road_frame(const sensor_msgs::PointCloud2ConstPtr& msg);  
  
  /**
   * \brief Calculate plane coefficients
   * It uses pcl RANSAC model to estimate the coeeficients of a plane
   * 
   * You may use after call yhis function: (to see Ax+By+Cz+D=0)
   * \code
   * cout<< coeeficients.values<<endl;
   * \endcode
   * \param[in] *ptin 
   * \param[in]  iterations
   * \param[in]  threshold
   * \param[out] coefficients
   * \return  cloud out
   */
  pcl::PointCloud<T> get_plane_coefficients(pcl::PointCloud<T> cloud_in, int iterations, float threshold, pcl::ModelCoefficients::Ptr coefficients);

  //variables definition
  float threshold;
  int iterations;
  bool debug;
  T point_origin;
  T point_x;
  T point_y;
  tf::TransformListener *listener_atc_ground_ptr; 
  tf::TransformBroadcaster *broadcast_ptr;
  std::string pub_frame_name;
  std::string reference_ground_frame;
  pcl::ModelCoefficients::Ptr coef;
  
  //Class Constructor  
  plane_model_road()
  {
    threshold=0.01;   //Initialize default values
    iterations=50;
    debug=false;
    
    point_origin.x=2; point_origin.y=0; point_origin.z=0;      
    point_x.x=4;      point_x.y=0;      point_x.z=0;
    point_y.x=4;      point_y.y=0;      point_y.z=0;
    pub_frame_name="/environment/road_frame";
    reference_ground_frame="/atc/vehicle/ground";
  };
  
  ~plane_model_road()
  {
    // Class Destructor
  };
  
private:
  
  // Variables definition
  pcl::PointCloud<T> outliers;
  T origin_projected;
  T pt_x_projected;
  T pt_y_projected;
  
  //     //Function declarations
  /**
   * \brief Project a point on a plane
   * It uses pcl projectinliers to project a point on a plane
   * \param[in]  *ptin 
   * \param[in]  coeff
   * \return  *ptout
   */      
  T project_point_to_plane(const T ptin, const pcl::ModelCoefficients::Ptr coeff);  
  
  /**
   * \brief Transform a pcl point between two ros frames
   * It uses a tf::Stamped transform to transform a point between reference frames
   * \param[in]  frame1
   * \param[in] frame2
   * \param[in]  pt_origin
   * \return point_transformed
   */
  T transform_pcl_point(std::string frame1, std::string frame2, T pt_origin);
  
  /**
   * \brief check if distance matches with threshold
   * If returned true, you probably get the road plane
   * \param[in] distance
   * \param[in] threshold
   * \return bool result
   */
  inline bool check_distance(double distance, double threshold)
  {
    if (distance<=threshold)
    {
      if (debug)
	cout << "PMS_DEBUG: ROAD FRAME FOUND" << endl;
      return true;
    }
    else
      return false;
  };
  
  
  /**
   * \brief find RP angles for road frame
   * Gets the road frame RP angles. Notice that you don't need the Y.
   * \param[in]  origin_projected
   * \param[in]  pt_x_projected
   * \param[in]  pt_y_projected
   * \return std::vector< double> with R and P.
   */
  vector<double> find_RP_angle(T origin_projected, T pt_x_projected, T pt_y_projected);  
  
  /**
   * \brief get the transformation point
   * Gets the road transform to publish 
   * \param[in]  origin_projected
   * \param[in]  pt_x_projected
   * \param[in]  pt_y_projected
   * \return transform from a projected point
   */
  tf::Transform get_transform(T origin_projected, T pt_x_projected, T pt_y_projected);
  
};

#include "plane_model_road_segmentation_implementation.hpp"

#endif
