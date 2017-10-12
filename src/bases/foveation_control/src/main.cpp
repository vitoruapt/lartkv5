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
 * @file main.cpp
 * @brief This file has the main functionalities of foveation control
 * @author Miguel Oliveira
 * @version v0
 * @date 2012-02-29
 */

#include <stdio.h>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
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
#include <sensor_msgs/JointState.h>
#include <pcl_conversions/pcl_conversions.h>

#define PFLN printf("file %s line %d\n",__FILE__,__LINE__);

#define _USE_DEBUG_ 0
//Global variables
ros::NodeHandle* p_n;
tf::TransformListener *p_listener;
double current_pan=0, current_tilt=0;
ros::Publisher pub;

void target_pose_cb(const sensor_msgs::PointCloud2ConstPtr & pcmsg_in)
{
	pcl::PointCloud<pcl::PointXYZ> pc_in;
	pcl::PointCloud<pcl::PointXYZ> pc_transformed;
	tf::StampedTransform transform;

	//p_listener->waitForTransform(pcmsg_in.header.frame_id, ros::names::remap("/tracking_frame"),ros::Time::now(), ros::Duration(0.2));
	try
	{
		p_listener->lookupTransform(pcmsg_in->header.frame_id, "/atc/ptu/tilt_block", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("Cannot find transform, returning");
	}

// 	pcl::fromROSMsg(*pcmsg_in,pc_in);

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*pcmsg_in, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc_in);
    
	pcl_ros::transformPointCloud(pc_in,pc_transformed,transform.inverse());
	pc_transformed.header.frame_id = "/atc/ptu/tilt_block";

	double target_x = pc_transformed.points[0].x;
	double target_y = pc_transformed.points[0].y;
	double target_z = pc_transformed.points[0].z;

	double delta_pan = atan2(target_y,target_x);
	double delta_tilt = atan2(target_z, sqrt(target_x*target_x+target_y*target_y));

	double new_pan = current_pan + delta_pan;
	double new_tilt = current_tilt + delta_tilt;

	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(2);
	joint_state.position.resize(2);

	joint_state.name[0] =ros::names::remap("pan");
	joint_state.position[0] = new_pan;

	joint_state.name[1] =ros::names::remap("tilt");
	joint_state.position[1] = new_tilt;

	pub.publish(joint_state);

}




	int main(int argc, char **argv)
	{

		ros::init(argc, argv, "foveation_control");
		ros::NodeHandle n("~");
		tf::TransformListener listener(n,ros::Duration(10));
		p_listener=&listener;
		p_n=&n;

		ros::Subscriber sub = n.subscribe("/target", 1, target_pose_cb);

		//declare the publisher
		pub = n.advertise<sensor_msgs::JointState>("/ptu_cmd", 1);

		ros::Rate loop_rate(10);
		ros::spin();

	}
