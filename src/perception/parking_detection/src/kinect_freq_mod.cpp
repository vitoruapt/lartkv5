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
 * @brief Modulation of the Kinect Hz of published pointclouds
 * @details Receive the point cloud from the Kinect sensor, but publishes only at a defined rate
 * @author Joel
 * @date 5-May-2012
 */

#include <stdio.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <points_from_volume/points_from_volume.h>
#define PFLN printf("FILE %s LINE %d\n",__FILE__, __LINE__);

// Namespaces
using namespace std;

// Global vars
ros::Publisher cloud_pub;
ros::NodeHandle* p_n;

/** 
 * @brief Point cloud treatment
 * @details This function cuts the point cloud at a certain distance
 * @param PointCloud message received from the Kinect sensor
 * @return void
 */
void conversion (const sensor_msgs::PointCloud2ConstPtr & pcmsg_in)
{
  pcl::PointCloud<pcl::PointXYZ> pc_cut;
  pcl::PointCloud<pcl::PointXYZ> processed_pc;
  sensor_msgs::PointCloud2 pcmsg_out;
  
  pcl::fromROSMsg(*pcmsg_in,pc_cut);
  pc_cut.header.frame_id=pcmsg_in->header.frame_id;

  for (int i=0;i<(int)pc_cut.points.size();i++)
  {
    if (pc_cut.points[i].z>0.75 && pc_cut.points[i].z<1.5)
    {
      processed_pc.push_back(pc_cut.points[i]);
    }
  }
  processed_pc.header.frame_id=pc_cut.header.frame_id;
  pcl::toROSMsg(processed_pc,pcmsg_out);
  
  pcmsg_out.header.stamp=pcmsg_in->header.stamp;
  cloud_pub.publish(pcmsg_out);
}



/** 
 * @brief Publish messages at a certain loop rate
 * @param int argc
 * @param char **argv
 * @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_freq_mod");                             
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  p_n=&n;
  
  // Subscribe of the Kinect point cloud message
  ros::Subscriber sub = n.subscribe ("/point_cloud_from_kinect", 1, conversion);
  
  // Advertise of the resutant point cloud
  cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/point_cloud_input", 1);
  
  while(ros::ok())
  {
	ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}
