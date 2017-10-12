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
 * \file
 * \brief plane_road_extraction class declaration
 */

#ifndef _PLANE_ROAD_EXTRACTION_H_
#define _PLANE_ROAD_EXTRACTION_H_

// ROS includes.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/surface/convex_hull.h>

// LAR includes
#include <points_from_volume/points_from_volume.h>
#include <plane_model_road_segmentation/plane_model_road_segmentation.h>

using namespace std;
using namespace pcl;

template<class T>
class plane_road_extraction
{
public:
    
    /// \brief class constructor
    plane_road_extraction()
    {
    };
    
    /// \brief class destructor
    ~plane_road_extraction()
    {
    };
    
    //  VARIABLES
    
    /// \brief Point cloud to filter
    PointCloud<T> pc_in;
    
    /// \brief  Road point cloud
    PointCloud<T> pc_road;
    
    /// \brief  Clusters point cloud
    PointCloud<T> pc_clusters;    
    
    /// \brief Class object for plane model road segmentation
    plane_model_road<T> pms;
    
    /// \brief Pointer to road publisher
    ros::Publisher * pub_road_ptr;

    /// \brief Pointer to clusters publisher
    ros::Publisher * pub_clusters_ptr;
    
    /// \brief Pointer to road perimeter publisher
    ros::Publisher * pub_road_perimeter_ptr;
    
    /// \brief Extraction limits to pointcloud
    double z_min,z_max;
    
    // FUNCTIONS
    
    /// \brief callback function
    void filter (const sensor_msgs::PointCloud2Ptr& msg);
    
    /**
     * \brief project a point cloud on a plane
     * 
     * \param[in] cloud_in
     * \param[in] coefficients
     * \return PointCloud< T> 
     */
    PointCloud<T> project_cloud_on_plane(PointCloud<T> cloud_in, ModelCoefficients::Ptr coefficients);
    
    /**
     * \brief transform pcl points between ros frames
     * 
     * \param[in] frame1    no need actually
     * \param[in] transform  the tf
     * \param[in] pt_origin  the point to transform
     * \return T the point transformed
     */
    T transform_pcl_point(string frame1, tf::Transform transform, T pt_origin);
    
};

#include "plane_road_extraction.hpp"

#endif
