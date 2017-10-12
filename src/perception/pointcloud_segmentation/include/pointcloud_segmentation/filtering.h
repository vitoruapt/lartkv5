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
 * \brief filtering class declaration
 */

#ifndef _FILTERING_H_
#define _FILTERING_H_

// ROS includes.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

// LAR includes
#include <points_from_volume/points_from_volume.h>

#ifndef PFLN
    #define PFLN printf("LINE= %d in FILE=%s \n", __LINE__, __FILE__);
#endif

using namespace std;
using namespace pcl;


/** \class filtering 
 A simple class to voxelize a pointcloud and cutt within a volume
 */
template<class T>
class filtering
{
public:
    
    /// \brief Class constructor 
    filtering()
    {
    };
    
    /// \brief Class destructor
    ~filtering()
    {
    };
    
    //  VARIABLES
    
    /// \brief Point cloud to filter
    PointCloud<T> pc_in;
    
    /// \brief  Point cloud filtered
    PointCloud<T> pc_out;
    
    /// \brief The convex hull for cloud extraction
    PointCloud<T> convex_hull;
    
    /// \brief Voxel grid size
    double voxel_size;
    
    /// \brief Maximum value for Z pointcloud
    double max_z;
    
    /// \brief Minimum value for Z pointcloud
    double min_z;
    
    /// \brief Pointer to pointcloud publisher
    ros::Publisher *pub_ptr;
    
    /// \brief Pointer to a transform listener
    tf::TransformListener *transform_listener_ptr;
    
    /// \brief The frame_id to transform
    string frame_id;
    
    // FUNCTIONS
        
    /// \brief Callback function for subscriber.
    void filter(const sensor_msgs::PointCloud2Ptr& msg);
        
    /**
     * \brief aplies a voxel grid on a sensor msg
     * 
     * \param[in] msg_in
     * \param[in] voxel_size
     * \return sensor_msgs::PointCloud2 
     */
    sensor_msgs::PointCloud2 voxel_filter(const sensor_msgs::PointCloud2Ptr& msg_in, double voxel_size);
       
};

#include "filtering.hpp"

#endif
