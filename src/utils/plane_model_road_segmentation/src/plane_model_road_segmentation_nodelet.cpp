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
#ifndef _plane_model_road_segmentation_CPP_
#define _plane_model_road_segmentation_CPP_

#include <stdio.h>
#include <plane_model_road_segmentation/plane_model_road_segmentation.h>
#include <plane_model_road_segmentation/coefficients.h>
#include "plane_model_road_segmentation_head.h"

#define PFLN printf("FUNCTION=%s  LINE=%d\n",__FUNCTION__,__LINE__);

/**
\file
\brief A simple nodelet to aply the function library
\author Tiago Talhada
**/

// Global declarations
tf::TransformListener *listener_center_bumper_ptr;
plane_model_road<pcl::PointXYZRGB> pms;
ros::Publisher pub_coeffs;

void topic_callback2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  static int counter=0; counter++;
  ROS_INFO("Message received number:%d",counter);
  
  // Pcl clouds
  pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
  pcl::fromROSMsg(*msg,cloud_in);
  
  tf::Transform road_tf=pms.find_and_publish_road_frame(cloud_in);  
  
  plane_model_road_segmentation::coefficients coefficients;
  coefficients.A=pms.coef->values[0];
  coefficients.B=pms.coef->values[1];
  coefficients.C=pms.coef->values[2];
  coefficients.D=pms.coef->values[3];
  pub_coeffs.publish(coefficients);
}


int main(int argc, char **argv)
{
  ROS_INFO("Starting plane_model_road_segmentation nodelet...");
  ros::init(argc,argv,"plane_segmentation_nodelet");
  
  ros::NodeHandle n;
  
  // Messages subscription configuration    
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  tf::TransformListener listener_atc_ground;      // atc/vehicle/ground
  listener_center_bumper_ptr=&listener_center_bumper;
  pms.listener_atc_ground_ptr=&listener_atc_ground;
  
  // Getting point cloud from launch file
  string pointcloud_subscribed;
  if (!n.hasParam("pointcloud_subscribed"))
  {
    ROS_ERROR("Must set properly a input pointcloud  (pointcloud_subscribed).");
    return -1;
  }
  else
  n.getParam("pointcloud_subscribed", pointcloud_subscribed);
  ros::Subscriber sub = n.subscribe(pointcloud_subscribed, 1, topic_callback2);
   
  // Publish a frame
  tf::TransformBroadcaster br;   // Road frame 
  pms.broadcast_ptr=&br;
  
  // Publish model coefficients
    string coefficients_topic_name;
  if (!n.hasParam("coefficients_topic_name"))
  {
    ROS_ERROR("Must set properly a coefficients_topic_name  (coefficients_topic_name).");
    return -1;
  }
  else
  n.getParam("coefficients_topic_name", coefficients_topic_name);
  pub_coeffs=n.advertise<plane_model_road_segmentation::coefficients>(coefficients_topic_name,1);
  
  
  if (!n.hasParam("road_frame"))
    ROS_INFO("publishing frame as /environment/road_frame");
  else
  n.getParam("road_frame",pms.pub_frame_name);
  
  if (!n.hasParam("reference_ground_frame"))
    ROS_WARN("You didn't set properly your ground reference frame. /atc/vehicle/ground is being used.");
  else
  n.getParam("reference_ground_frame",pms.reference_ground_frame);
  
  ros::spin();
  
  return 1;
}
#endif
