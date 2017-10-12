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
 * \brief region_growing_node class main declaration
 */

#ifndef _REGION_GROWING_NODE_H_
#define _REGION_GROWING_NODE_H_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros.h>

// LAR
#include <pointcloud_segmentation/region_growing.h>
#include <pointcloud_segmentation/rviz_markers.h>

using namespace std;
using namespace pcl;

template<class T, class T1>
class region_growing_node
{
public:
    
    /// \brief class constructor
    region_growing_node()
    {
    };
    
    /// \brief class destructor
    ~region_growing_node()
    {
    };
    
    // VARIABLES
    
    /// \brief Laser input point cloud
    PointCloud<T1> laser_scan;
    
    /// \brief A publisher pointer to a marker
    ros::Publisher * pub_markers_ptr;
    
    /// \brief A publisher pointer to add some text in clusters
    ros::Publisher * pub_markers_text_ptr;
    
    /// \brief Minimum points to consider the cluster
    double min_cluster_size;
    
    /// \brief Maximum points to consider the cluster
    double max_cluster_size;
    
    /// \brief Tolerance between points on the same cluster
    double clustering_tolerance;
    
    /// \brief The radius limit for region growing algorithm
    double radius;
    
    // FUNCTIONS
    
    /// \brief callback for point cloud
    void callback_cloud (const sensor_msgs::PointCloud2Ptr& msg);
    
    /// \brief callback for laser cloud
    void callback_laser (const sensor_msgs::PointCloud2Ptr& msg);
    
};

#include "region_growing_node.hpp"

#endif
