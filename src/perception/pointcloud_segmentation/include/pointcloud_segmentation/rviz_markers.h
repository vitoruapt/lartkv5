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
 * \brief markers class main declaration
 */

#ifndef _RVIZ_MARKERS_H_
#define _RVIZ_MARKERS_H_

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
using namespace std;
using namespace pcl;


template<class T>
class markers
{
public:    
    
    /** 
     * \brief function to estimate a marker arround the cluster to show on rviz
     * \param[in] cloud_cluster pointer to the cluster
     * \param[in] ns string with the classification name
     * \param[in] id the marker ident
     * \param[in] r the r channel color
     * \param[in] g the g channel color
     * \param[in] b the b channel color
     * \return marker the marker to publish
     */
    visualization_msgs::Marker mark(typename PointCloud<T>::Ptr cloud_cluster, string ns ,int id, float r, float g, float b);
    
    /** 
     * \brief function to estimate a marker arround the cluster to show on rviz
     * This function will set a marker very far away and very small. This was the way i find to automatically delete a marker. :\
     * \param[in] frame_id string with the frame_id
     * \param[in] ns string with the classification name
     * \param[in] id the marker ident
     * \return marker the marker to publish
     */
    visualization_msgs::Marker unmark(std::string frame_id, std::string ns, int id);
   
    /** 
     * \brief function to create a marker with some text info about a pointcloud (cluster)
     * \param[in] cloud_cluster the pointcloud input data points
     * \param[in] ns string with the classification name
     * \param[in] id the marker id
     * \param [in] rgb colors for text
     * \return marker the marker to publish
     */
    visualization_msgs::Marker mark_text(typename PointCloud<T>::Ptr cloud_cluster, string ns ,int id, float r, float g, float b);
    
    
};

#include "rviz_markers.hpp"

#endif
