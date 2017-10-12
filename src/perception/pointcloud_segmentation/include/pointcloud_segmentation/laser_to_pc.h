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
 * \brief laser_to_pc class declaration
 */

#ifndef _LASER_TO_PC_H_
#define _LASER_TO_PC_H_

// ROS includes.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/conversions.h>

// LAR includes
#include <points_from_volume/points_from_volume.h>

using namespace std;
using namespace pcl;


class laser_to_pc
{
public:  
    /// \brief class constructor
    laser_to_pc()
    {
    };
    
    /// \brief class destructor
    ~laser_to_pc()
    {
    };
   
    // VARIABLES
    
    /// \brief publisher pointer
    ros::Publisher *pub_ptr;
    
    /// \brief frame id
    string frame_id;
    
    /// \brief z limits for extraction
    double min_z,max_z;
    
    /// \brief transform_listener pointer
    tf::TransformListener *listener_ptr;
   
    /// \brief convex_hull for extraction
    PointCloud<PointXYZ> convex_hull;
    
    // FUNCTIONS
    
   /// \brief callback function
   void filter (const sensor_msgs::LaserScanPtr& msg);
  
};


#endif

