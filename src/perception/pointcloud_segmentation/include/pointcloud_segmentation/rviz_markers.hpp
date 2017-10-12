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
 * \brief markers class functions implementation
 */

#ifndef _RVIZ_MARKERS_HPP_
#define _RVIZ_MARKERS_HPP_

#include <pointcloud_segmentation/rviz_markers.h>

/** \brief convert numeric type to string 
 * This function should work for all numeric c++ types: int, double, float....*/
template <class T>
std::string to_string (const T& t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
};
////////////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
visualization_msgs::Marker markers<T>::unmark(std::string frame_id, std::string ns, int id)
{
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    
    marker.pose.position.x = 100;
    marker.pose.position.y = 100;
    marker.pose.position.z = 100;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    
    marker.ns = ns;
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    return marker;
}; 
//////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
visualization_msgs::Marker markers<T>::mark(typename PointCloud<T>::Ptr cloud_cluster, string ns ,int id, float r, float g, float b)
{
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    
    compute3DCentroid (*cloud_cluster, centroid);
    getMinMax3D (*cloud_cluster, min, max);
    
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_cluster->header.frame_id;
    marker.header.stamp = ros::Time::now();
    
    marker.ns = ns;
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = (max[0]-min[0]);
    marker.scale.y = (max[1]-min[1]);
    marker.scale.z = (max[2]-min[2]);
    
    if (marker.scale.x ==0)
	marker.scale.x=0.1;
    
    if (marker.scale.y ==0)
	marker.scale.y=0.1;
    
    if (marker.scale.z ==0)
	marker.scale.z=0.1;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.3;
    
    marker.lifetime = ros::Duration();
    //   marker.lifetime = ros::Duration(0.5);
    return marker;
};
////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
visualization_msgs::Marker markers<T>::mark_text(typename PointCloud<T>::Ptr cloud_cluster, string ns ,int id, float r, float g, float b)
{
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    
    compute3DCentroid (*cloud_cluster, centroid);
    pcl::getMinMax3D (*cloud_cluster, min, max);
    
    uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_cluster->header.frame_id;
    marker.header.stamp = ros::Time::now();
    
    marker.ns = ns;
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = max[2]+0.2;
    
    marker.scale.z = 0.4;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    
    // Setup the text to show
    string text;
    text = "Cluster" + to_string(id) + "\n X=" + to_string(round(centroid[0])) + ", Y=" + to_string(round(centroid[1])) + ", Z=" + to_string(round(centroid[2])) ;
    marker.text=text;
    marker.lifetime = ros::Duration();   
    return marker;
};
////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
