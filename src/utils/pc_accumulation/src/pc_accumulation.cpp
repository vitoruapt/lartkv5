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
#include <pc_accumulation/pc_accumulation.h>
#include <pcl/filters/voxel_grid.h>
//#include <fstream>
//#include <iostream>

/**
  \file pc_accumulation.cpp
  \brief Library functions for PointCloud Accumulation

  This library provides a stack of function for the accumulation of pointclouds.
  \author Pedro Salvado
 */


//voxel_filter
sensor_msgs::PointCloud2 voxel_filter(sensor_msgs::PointCloud2 msg_in, float voxel_size)
{
    ROS_ERROR("Voxel grid does not work with sensor_msgs::PointCloud2, convertion is needed\n");
// 	sensor_msgs::PointCloud2 msg_out;
// 	sensor_msgs::PointCloud2ConstPtr msg_ptr (new sensor_msgs::PointCloud2 (msg_in));
// 	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
// 	sor.setInputCloud(msg_ptr);
// 	sor.setLeafSize (voxel_size, voxel_size, voxel_size);
// 	sor.filter (msg_out);
// 	return msg_out;
}


/**
 * \brief Main Function to accumulate pointCloud
 * This function receives a pcl pointcloud and the accumulation frame.\n
 * It <i>looks</i> for a transformation between the pcl frame_id and the accumulation frame_id.\n
 * Proceeds with the transformation, and accumulates it on the class variable pcl_pc_acc
 * \param[in]  pcl::PointCloud<pcl::PointXYZRGB>
 * \param[in]  std::string
 * \return transform from a projected point
 */

int pc_accumulation::pointcloud_accumulated(pcl::PointCloud<pcl::PointXYZRGB> pcl_pc,std::string acc_frame_id)
{
	tf::StampedTransform transform;

// 	pcl_pc.header.stamp = ros::Time();
    ROS_ERROR("Point cloud stamp not set\n");
    
	pcl::PointCloud<pcl::PointXYZRGB> pcl_transformed;

	bool have_transform=false;
	try
	{
// 		p_listener->lookupTransform(acc_frame_id,pcl_pc.header.frame_id , pcl_pc.header.stamp, transform);
        p_listener->lookupTransform(acc_frame_id,pcl_pc.header.frame_id , ros::Time::now(), transform);
        ROS_ERROR("Point cloud stamp not set\n");
        
		have_transform=true;
	}
	catch (tf::TransformException ex)
	{
// 		if(p_listener->waitForTransform(acc_frame_id,pcl_pc.header.frame_id,pcl_pc.header.stamp, ros::Duration(3.0)))
// 		{
            
        ROS_ERROR("Point cloud stamp not set\n");
        
        if(p_listener->waitForTransform(acc_frame_id,pcl_pc.header.frame_id,ros::Time::now(), ros::Duration(3.0)))
        {
			try
			{
                p_listener->lookupTransform(acc_frame_id,pcl_pc.header.frame_id , ros::Time::now(), transform);
                
                ROS_ERROR("Point cloud stamp not set\n");
                
// 				p_listener->lookupTransform(acc_frame_id,pcl_pc.header.frame_id , pcl_pc.header.stamp, transform);
				have_transform=true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("Could not lookup transform after waiting 3 secs\n.%s",ex.what());
			}
		}
		else
		{
			ROS_ERROR("Could find valid transform after waiting 3 secs\n.%s",ex.what());
		}
	}

	if (have_transform==false)
	{
		return 0;	
	}
	
	// Transform pc1 to pc2 ->transform
	pcl_ros::transformPointCloud(pcl_pc,pcl_transformed,transform);
	pcl_transformed.header.frame_id=acc_frame_id;
       
	pcl_pc_acc+=pcl_transformed;
    
	sensor_msgs::PointCloud2 pc_trans;
	sensor_msgs::PointCloud2 pc_voxel;

	pcl::toROSMsg(pcl_pc_acc,pc_trans);

	pc_voxel=voxel_filter(pc_trans,voxel_size);
	pcl::fromROSMsg(pc_voxel,pcl_pc_acc);

	pcl_pc_acc.header.frame_id=acc_frame_id;
	return 1;
}

/**
 * \brief Removes points of pointclouds, given a distance and remove_frame.
 * This function calculates the distance between each pcl_point and the remove_frame origin.\n
 * If the point is farther then the desired distance, it is removed from the pcl.
 * \param[in]  pcl::PointCloud<pcl::PointXYZRGB>
 * \param[in]  dist
 * \param[in]  X0
 * \param[in]  Y0
 * \param[in]  Z0
 */

//ofstream myfile;
int pc_accumulation::remove_points_from_pointcloud(pcl::PointCloud<pcl::PointXYZRGB> pcl_pc,float dist, float X0, float Y0, float Z0)
{
/*	int counttt=0,counttttt=0;

	static int init=0;
	if (init==0)
	{
		init=1;
        	myfile.open ("../percentagem.txt");
	}
*/

// 	ROS_INFO("before cleanup %ld points", pcl_pc.points.size());
	for(long int i=pcl_pc.points.size()-1; i>0; --i)
	{
		float dist_x, dist_y, dist_z, dist_norm;

		dist_x=pcl_pc.points[i].x-X0;
		dist_y=pcl_pc.points[i].y-Y0;
		dist_z=pcl_pc.points[i].z-Z0;
        
		dist_norm=sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
		//verificar qto pontos se encontram
	/*	if (sqrt(dist_x*dist_x)> 8 || (sqrt(dist_y*dist_y)> 2 || sqrt(dist_z*dist_z)> 0.2))
		{
			pcl_pc.erase(pcl_pc.points.begin()+i);
			ROS_INFO("after cleanup %ld points", pcl_pc.points.size());
		}		
		if(( (dist_x>0 &&dist_x<5) || sqrt(dist_y*dist_y)< 0.5) && sqrt(dist_z*dist_z)< 0.2)
		{
			counttttt++;
			if ( dist_z< 0.10 && dist_z>-0.10)
				counttt++;
		}*/

		if(dist_norm>dist) //verify distance
		{
			pcl_pc.erase(pcl_pc.points.begin()+i);
		}
	}
	pcl_pc_acc=pcl_pc;

	return 1;
}



