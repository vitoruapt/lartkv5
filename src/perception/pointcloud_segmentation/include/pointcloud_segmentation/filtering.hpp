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

#ifndef _FILTERING_HPP_
#define _FILTERING_HPP_

#include "filtering.h"

template <class T>
void filtering<T>::filter(const sensor_msgs::PointCloud2Ptr& msg)
{
    ROS_ERROR("Voxel filtering dissabled, must correct for ROS::HYDRO");
//     /// Pass through a square voxel grid
//     sensor_msgs::PointCloud2 msg_filt;
//     msg_filt = voxel_filter(msg,voxel_size);
//     
//     /// Transform to atc/center/bumper
//     PointCloud<T> cloud_voxelized;
//     fromROSMsg(msg_filt,cloud_voxelized);
//   
//     tf::StampedTransform transform;
//     try
//     {
// 	transform_listener_ptr->lookupTransform(frame_id, msg->header.frame_id, ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex)
//     {
// 	ROS_ERROR("%s",ex.what());
//     }
// 
//     PointCloud<T> cloud_trans;
//     pcl_ros::transformPointCloud (cloud_voxelized,cloud_trans,transform);
//     cloud_trans.header.frame_id=frame_id;
//     
//     /// Cutting the point cloud
//     points_from_volume<T> pfv;
//     PointCloud<T> cloud_cut;
//     cloud_cut.header.frame_id=frame_id;
//     pfv.set_convex_hull(convex_hull);
//     pfv.convexhull_function(cloud_trans, max_z,min_z, false);
//     cloud_cut=pfv.get_pc_in_volume();
//     
//     toROSMsg(cloud_cut,msg_filt);
//         
//     /// Publish the result
//     pub_ptr->publish(msg_filt);
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
sensor_msgs::PointCloud2 filtering<T>::voxel_filter(const sensor_msgs::PointCloud2Ptr& msg_in, double voxel_size)
{
    ROS_ERROR("Voxel filtering dissabled, must correct for ROS::HYDRO");
//     sensor_msgs::PointCloud2 msg_out;
//     VoxelGrid<sensor_msgs::PointCloud2> sor;
// //     sensor_msgs::PointCloud2ConstPtr batatas (new sensor_msgs::PointCloud2 (msg_in));
//     sor.setInputCloud(msg_in);
//     sor.setLeafSize (voxel_size, voxel_size, voxel_size);
//     sor.filter (msg_out);
//     return msg_out;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
