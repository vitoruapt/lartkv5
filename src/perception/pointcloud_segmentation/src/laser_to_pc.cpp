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
/** \brief A simple implementation to convert frames and transform a laser scan on a point cloud
 *  \file
 *  \author Tiago Talhada
 *  \date June 2012
 */
#include <pointcloud_segmentation/laser_to_pc.h>

void laser_to_pc::filter (const sensor_msgs::LaserScanPtr& msg)
{
    PointCloud<PointXYZ> laser_cloud;
    sensor_msgs::PointCloud2 laser;  // laser pointcloud out
    laser.header.frame_id=msg->header.frame_id;
    tf::TransformListener listener;  // not sure why i need this but it is ok :-)
        
    tf::StampedTransform transform;
    try
    {
	listener_ptr->lookupTransform(frame_id, msg->header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
	ROS_ERROR("%s",ex.what());
    }
        
    laser_geometry::LaserProjection projector_; 
    projector_.transformLaserScanToPointCloud(msg->header.frame_id,*msg,laser,listener);
    
    fromROSMsg(laser,laser_cloud);
    laser_cloud.header.frame_id=msg->header.frame_id;
    
    // Transform laser scan
    PointCloud<PointXYZ> laser_cloud_transformed;
    pcl_ros::transformPointCloud (laser_cloud,laser_cloud_transformed,transform);
    
    // Cut the cloud
    points_from_volume<PointXYZ> pfv;
    PointCloud<PointXYZ> cloud_cut;
    cloud_cut.header.frame_id=frame_id;
    convex_hull.header.frame_id=frame_id;
    laser_cloud_transformed.header.frame_id=frame_id;
    pfv.set_convex_hull(convex_hull);
    pfv.convexhull_function(laser_cloud_transformed, max_z,min_z, false);
    cloud_cut=pfv.get_pc_in_volume();
        
    sensor_msgs::PointCloud2 msg_laser;
    toROSMsg(cloud_cut, msg_laser);
    
    // Publish the result
    pub_ptr->publish(msg_laser);
    
};
