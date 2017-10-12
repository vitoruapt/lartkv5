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
#ifndef _points_from_volume_lib_H_
#define _points_from_volume_lib_H_

/**
 * @file
 * @brief points_from_volume class declaration and auxiliary types
 * @author Joel Pereira
 * @version v0
 * @date 2012-03-26
 */

//////////////////////////////////////////////////////////
//INCLUDES
//////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>
typedef enum {SUCCESS, FAILURE}t_func_output;

#define _CONVEX_LIB_DEBUG_ 0

using namespace std;

template<class T>
class points_from_volume
{

public:

	points_from_volume()
	{
		flag_convexhull_set=false;
	};

	~points_from_volume()
	{
#if _CONVEX_LIB_DEBUG_ 
printf("Destructor...\n");
#endif
	};

	
	/** 
	* @brief Set the convexhull 
	* @details Copy the points from a point cloud to the convexhull  
	* @param pcl::PointCloud<T>& pc_in
	* @return t_func_output
	*/
	t_func_output set_convex_hull(pcl::PointCloud<T>& pc_in)
	{
		convex_hull=pc_in;
		convex_hull.header.frame_id = pc_in.header.frame_id;
		#if _CONVEX_LIB_DEBUG_
			cout<<"pc_in frame id on set_convex_hul -> "<<pc_in.header.frame_id<<endl;
		#endif
		flag_convexhull_set=true;
		return SUCCESS;
	};
	
	/** 
	* @brief Set the convexhull 
	* @details Copy the points from a point cloud to the convexhull  
	* @param const sensor_msgs::PointCloud2ConstPtr& pcmsg_in
	* @return set_convex_hull(pc_in);
	*/
	t_func_output set_convex_hull(const sensor_msgs::PointCloud2ConstPtr& pcmsg_in)
	{
		pcl::PointCloud<T> pc_in;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*pcmsg_in, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, pc_in);;
		return set_convex_hull(pc_in);
	};
	
	/** 
	* @brief Points in the desired volume 
	* @param void
	* @return pc_in_volume
	*/
	pcl::PointCloud<T> get_pc_in_volume()
	{
		return pc_in_volume;
	};
	
	/** 
	* @brief Extract points from the pcl that are inside the convexhull 
	*
	* There is a need to set some convexhull parameters, like the offsets (positive and negative) and the flag which defines if the points are extrated from the inside(flag=false) or the outside(flag=true) of the hull 
	* @param pc_in_volume
	* @param positive_offset
	* @param negative_offset
	* @param flag_in_out
	* @return t_func_output
	*/
 	t_func_output convexhull_function(pcl::PointCloud<T>& pc_in, double positive_offset, double negative_offset, bool flag_in_out)
	{
		if(!flag_convexhull_set)
		{
			ROS_ERROR("Flag not set to TRUE");
			return FAILURE;
		}
		
		if(pc_in.header.frame_id!=convex_hull.header.frame_id)
		{
#if _CONVEX_LIB_DEBUG_
cout<<"convex_hull frame id "<<convex_hull.header.frame_id<<endl;
cout<<"pc_in frame id on convexhull_function"<<pc_in.header.frame_id<<endl;
#endif
			ROS_ERROR("Non-matching frame id's");
			return FAILURE;
		}
		
		pcl::ExtractPolygonalPrismData<T> epp;                                
		typename pcl::PointCloud<T>::ConstPtr input_cloud_constptr;             
		input_cloud_constptr.reset (new pcl::PointCloud<T> (pc_in));
		typename pcl::PointCloud<T>::ConstPtr convex_hull_constptr;
		convex_hull_constptr.reset (new pcl::PointCloud<T> (convex_hull));

		pcl::ExtractIndices<T> extract; //Create the extraction object
		pcl::PointIndices::Ptr indices;
		indices.reset();
		indices = pcl::PointIndices::Ptr(new pcl::PointIndices); 

		pc_in_volume.header.frame_id=pc_in.header.frame_id;
		
		//Set epp parameters
		epp.setInputCloud(input_cloud_constptr);
		epp.setInputPlanarHull(convex_hull_constptr);
		epp.setHeightLimits(negative_offset,positive_offset); 
		epp.setViewPoint(0,0,0); //i dont think this serves any purpose in the case of epp
		epp.segment(*indices);
		
		pc_in_volume.points.clear();
		pc_in_volume.height=1;
		pc_in_volume.width=pc_in_volume.points.size();
		if ((int)indices->indices.size()!=0)
		{
			extract.setInputCloud(pc_in.makeShared());
			extract.setIndices(indices);
			extract.setNegative(flag_in_out);
			extract.filter(pc_in_volume);
		}
// 		else
// 		{
// 			extract.setInputCloud(pc_in.makeShared());
// 			extract.setIndices(indices);
// 			extract.setNegative(flag_in_out);
// 			extract.filter(pc_in_volume);
// 			
// 		cout<<"NUMBER OF POINTS: "<<pc_in_volume.points.size()<<endl;
// 		}
// cout<<"NUMBER OF POINTS: "<<pc_in_volume.points.size()<<endl;
			input_cloud_constptr.reset();
			convex_hull_constptr.reset();
			indices.reset();
		return SUCCESS;
	}
	

private:
	pcl::PointCloud<T> pc_in_volume;
	bool flag_convexhull_set;
	pcl::PointCloud<T> convex_hull;
};


#endif
