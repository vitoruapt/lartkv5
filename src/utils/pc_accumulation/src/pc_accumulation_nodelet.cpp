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
 \file pc_accumulation_nodelet.cpp
 \brief Nodelet that use pc_accumulation class
 * This nodelet subscribes sensor_msgs::PointCloud2 and sensor_msgs::LaserScanPtr
 * Accumulate on the desired frame_id (see pc_accumulation.lauch)
 \author Pedro Salvado
 **/
#define PFLN {printf("%s %d\n",__FILE__, __LINE__);}

#include <pc_accumulation/pc_accumulation.h>
//GLOBAL VARIABLES
ros::Publisher cloud_pc_laserScan;
ros::Publisher cloud_pc_pointcloud;
pc_accumulation *p_lib;
double distance_from, timer_value;
std::string laser_topic,pc_topic,acc_frame;


bool process_laser_scan[5]={false,false,false,false,false};
bool process_point_cloud[5]={false,false,false,false,false};


/**
 * \brief Callback from the Odometry subscribed topic
 * \param[in]  const nav_msgs::Odometry
 */
void OdometryCallBack (const nav_msgs::OdometryConstPtr & odom_in)
{
       p_lib->odometry_=*odom_in;   
}

/**
 * \brief Callback from the PointCloud subscribed topic
 * \param[in]  const sensor_msgs::PointCloud2
 */
void pointcloud_Callback(const sensor_msgs::PointCloud2Ptr& image)
{
       p_lib->pointcloud_accumulated(*image,acc_frame);
}

/**
 * \brief Callback from the LaserScan subscribed topic
 * \param[in]  scan_in
 */
void laserscan_Callback(const sensor_msgs::LaserScanPtr& scan_in)
{
       p_lib->pointcloud_accumulated(*scan_in,acc_frame);
}


/**
 * \brief Function responsible for Remove and Publish PointCloud ()
 * \param[in]  ros::TimerEvent
 */
void Remove_Publish(const ros::TimerEvent&)
{
	sensor_msgs::PointCloud2 msg; 
	p_lib->remove_points_from_pointcloud(p_lib->pcl_pc_acc,distance_from, acc_frame);
	pcl::toROSMsg(p_lib->pcl_pc_acc,msg);
	cloud_pc_pointcloud.publish(msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "nodelet"); // Initialize ROS coms
	ros::NodeHandle n("~"); //The node handle
// 	     T=ros::Time::now();
	//Check topic remapping
	//Test the remapping of the /laserscan s
	for (int i=0; i<5; i++)
	{	
		char str[1024];
		sprintf(str,"/laserscan%d", i);
		if (ros::names::remap(str)!=str)
		{
			process_laser_scan[i] = true;
		}
	}

	//Test the remapping of the /pointcloud s
	for (int i=0; i<5; i++)
	{	
		char str[1024];
		sprintf(str,"/pointcloud%d", i);
		if (ros::names::remap(str)!=str)
		{
			process_point_cloud[i] = true;
		}
	}
	std::vector<ros::Subscriber> sub;
	//Creates ROS subscribers if the laserscan topics where remapped
	for (int i=0; i<5; i++)
	{
		if (process_laser_scan[i]==true)
		{
			char str[1024];
			sprintf(str,"/laserscan%d", i);
			ros::Subscriber sub_ = n.subscribe (str, 1, laserscan_Callback);
			ROS_INFO("Subscribing to %s", (ros::names::remap(str)).c_str());
			sub.push_back(sub_);
		}
	}

	//Creates ROS subscribers if the pointcloud topics where remapped
	for (int i=0; i<5; i++)
	{
		if (process_point_cloud[i]==true)
		{
			char str[1024];
			sprintf(str,"/pointcloud%d", i);
			ros::Subscriber sub_ = n.subscribe (str, 1, pointcloud_Callback);
			ROS_INFO("Subscribing to %s", (ros::names::remap(str)).c_str());
			        ROS_INFO("REcebeu pointcloud");
			sub.push_back(sub_);
		}
	}


	if (sub.size()==0)
	{
		ROS_ERROR("No /laserscan[0 4] or /pointcloud[0 4] where remapped. Aborting...");
		ros::shutdown();
	}


	//Check param remapping
	if (!n.hasParam("acc_frame")) //
	{
		ROS_ERROR("Didn't introduce a accumulation frame");
		return -1;
	}
	else
	{
		n.getParam("acc_frame",acc_frame);
	}


	//verify accumulation parameters 
	if (!n.hasParam("distance_from"))
		ROS_WARN("distance for accumulation 15 meters");
	else
		n.param("distance_from",distance_from, 15.0);

	if (!n.hasParam("timer_value"))
		ROS_WARN("Frequency of publication 2.0 sec");
	else
		n.param("timer_value",timer_value, 2.0);

	
	
              // Declaration class
              pc_accumulation lib;
	     
              
	if (!n.hasParam("voxel_size"))
		 ROS_WARN("Voxel_size Default 0.4 meter");
        else
		 n.param("voxel_size",lib.voxel_size, 0.4);
	//remap string
	string removed_from;
	if (!n.hasParam("removed_from"))
	{
	            ROS_ERROR("Remove from frame not Defined");
	            return -1;
	}
	else
	            n.getParam("removed_from",removed_from);

	lib._acc_frame_id=removed_from;

	//Pointer to lib
	p_lib=&lib;
	
	//Remove&Publish accumulated pointcloud
	ros::Timer timer = n.createTimer(ros::Duration(timer_value), Remove_Publish);

	tf::TransformListener listener(n,ros::Duration(10));
	// Transform Listener
	lib.p_listener=&listener;
	//  Node Handle
	lib.p_n=&n;

	cloud_pc_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/pc_out_pointcloud", 1);
	ros::Subscriber odom = n.subscribe ("odometry_topic", 1, OdometryCallBack);


	ros::Rate loop_rate(50);
	ros::spin();


	return 0;
}
