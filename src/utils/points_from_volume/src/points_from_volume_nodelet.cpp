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
#ifndef _points_from_volume_nodelet_CPP_
#define _points_from_volume_nodelet_CPP_
/**
 * @file
 * @brief The main file, nodelet implementation code 
 * @author Joel Pereira
 * @version v0
 * @date 2011-03-26
 */
#include <ros/ros.h>
#include <points_from_volume/points_from_volume.h>

// Global Vars
ros::NodeHandle* p_n;
ros::Publisher cloud_pub;
bool defined_hull=false;
double positive;
double negative;
bool flag;
// Class member
points_from_volume<pcl::PointXYZ> piv;

/** 
 * @brief Point Cloud callback 
 * @details This callback makes use of the convexhull class created  
 * @param const sensor_msgs::PointCloud2ConstPtr & pcmsg_in
 * @return void
*/
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr & pcmsg_in)
{
    pcl::PointCloud<pcl::PointXYZ> pc_in;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*pcmsg_in, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc_in);
    
    pcl::PointCloud<pcl::PointXYZ> pc_filtered;
    piv.convexhull_function(pc_in, positive, negative, flag);
    sensor_msgs::PointCloud2 pcmsg_out;
    pcl::toROSMsg(piv.get_pc_in_volume(), pcmsg_out);
    // Uncomment the next line to print the point cloud size
    // std::cout<<"Num points "<<pc_filtered.points.size()<<std::endl;
    cloud_pub.publish(pcmsg_out);
}

/** 
 * @brief ConvexHull callback 
 * @details This callback sets a flag to true (when the convexhull is received), allowing the rest of the code to run  
 * @param const sensor_msgs::PointCloud2ConstPtr & pcmsg_convexhull_in
 * @return void
*/
void cb_points_convexhull (const sensor_msgs::PointCloud2ConstPtr & pcmsg_convexhull_in)
{
	printf("-------Flag is now True-------\n");
	piv.set_convex_hull(pcmsg_convexhull_in);
	defined_hull=true;
}


/** 
 * @brief Main code of the nodelet
 *
 * Initializes params and receive and pubish pointclouds  
 * @param argc
 * @param argv
 * @return int
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_from_volume_nodelet"); 
	ros::NodeHandle n("~"); 
// 	tf::TransformListener listener(n,ros::Duration(10));
// 	p_listener=&listener;
	p_n=&n;
	
	// Set default values on parameters
	if(!positive)
	{
		ROS_WARN_ONCE("Positive_offset not defined. Setting to default value '2.0'\n");
		n.param("positive_offset", positive, 2.0);
	}
	if(!negative)
	{
		ROS_WARN_ONCE("Negative_offset not defined. Setting to default value '-0.1'\n");
		n.param("negative_offset", negative, -0.1);
	}
	if(!flag)
	{
		ROS_WARN_ONCE("Flag_in_out not defined. Setting to default value 'false'\n");
		n.param("flag_in_out", flag, false);
	}
	
	// Point clouds subscription
	ros::Subscriber sub1 = n.subscribe ("/points_convexhull", 1, cb_points_convexhull);
	ros::Subscriber sub2 = n.subscribe ("/point_cloud_input", 1, cloud_cb);  
	
	if(defined_hull==false)
	{
		ROS_WARN("No point cloud convexhull received yet...");
	}
	
	// Pubishes the point cloud
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/pcmsg_out", 1);
	
	ros::Rate loop_rate(30);
	ros::spin();
}
#endif
