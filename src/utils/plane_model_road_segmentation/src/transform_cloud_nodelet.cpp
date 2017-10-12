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
\file 
\brief A simple nodelet to transform a point cloud
\author Tiago Talhada
**/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#define PFLN printf("Line=%d\n", __LINE__);

using namespace std;

// Declare a publisher

ros::Publisher pub;
string frame_id;
tf::TransformListener *listener_center_bumper_ptr;


void topic_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  static int counter=0;  counter++;
  ROS_INFO("Message received n= %d",counter);
  
  // Pcl clouds
  pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
  
  //STEP 0 Convert sensor_msgs to pcl
//   pcl::fromROSMsg(*msg,cloud_in);
  
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud_in);
  
  //STEP 1 Convert xb3 message to center_bumper frame (i think it is better this way)
  tf::StampedTransform transform;
  try
  {
    listener_center_bumper_ptr->lookupTransform(frame_id, msg->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // Transform point cloud
  pcl_ros::transformPointCloud (cloud_in,cloud_trans,transform);  
  cloud_trans.header.frame_id=frame_id;  // Must set new frame manually
  sensor_msgs::PointCloud2 msg_pub;  
  
  pcl::toROSMsg(cloud_trans, msg_pub);
  pub.publish(msg_pub);
}


int main(int argc, char **argv)
{
  ROS_INFO("Started transform cloud nodelet");
  ros::init(argc,argv,"transform_cloud_nodelet");

  ros::NodeHandle n;
  
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  listener_center_bumper_ptr=&listener_center_bumper;
  
  // Getting point cloud from launch file
  string pointcloud_subscribed;
  if (!n.hasParam("pointcloud_to_transform"))
  {
    ROS_ERROR("Must set properly a input pointcloud");
    return -1;
  }
  else
  n.getParam("pointcloud_to_transform", pointcloud_subscribed);
  
  // Getting reference frame to transform
  if (!n.hasParam("frame_id"))
  {
    ROS_ERROR("Must set properly a frame_id to transform");
    return -1;
  }
  else
  n.getParam("frame_id", frame_id);
  
  // Getting reference frame to transform
  string pointcloud_transformed;
  if (!n.hasParam("pointcloud_transformed"))
  {
    ROS_ERROR("Must set properly a topic to publish the transformed cloud");
    return -1;
  }
  else
  n.getParam("pointcloud_transformed", pointcloud_transformed);

  ros::Subscriber sub = n.subscribe(pointcloud_subscribed, 1, topic_callback);
  pub=n.advertise<sensor_msgs::PointCloud2>(pointcloud_transformed,1000);
  
  ros::spin();
  return 1;


}
