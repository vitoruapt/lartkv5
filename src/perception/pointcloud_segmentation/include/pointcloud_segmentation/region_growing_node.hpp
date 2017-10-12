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
 * \brief region_growing_node class functions implementation
 */

#ifndef _REGION_GROWING_NODE_HPP_
#define _REGION_GROWING_NODE_HPP_

#include <pointcloud_segmentation/region_growing_node.h>

template<class T, class T1>
void region_growing_node<T,T1>::callback_cloud (const sensor_msgs::PointCloud2Ptr& msg)
{
    typename PointCloud<T>::Ptr cloud (new PointCloud<T> ());
    typename PointCloud<T1>::Ptr laser_scan_ptr (new PointCloud<T1> (laser_scan));
    
    fromROSMsg(*msg,*cloud);
           
    /// Step 1 Cluster the laser scan 
    EuclideanClusterExtraction<T1> ec_inside;
    vector<PointIndices> indices;
    ec_inside.setClusterTolerance (clustering_tolerance);
    ec_inside.setMinClusterSize (min_cluster_size);
    ec_inside.setMaxClusterSize (max_cluster_size);
    ec_inside.setInputCloud (laser_scan_ptr);
    ec_inside.extract (indices);
    
    // Create markers class
    markers<PointXYZRGB> mk;
    
    static int marker_id;
    // Delete all previous markers
    if (marker_id>0)
    {
	for (int i=0; i<=marker_id; ++i)
	{
	    visualization_msgs::Marker marker_eraser=mk.unmark(msg->header.frame_id, "Obstacle", i);
	    pub_markers_ptr->publish(marker_eraser);
	    pub_markers_text_ptr->publish(marker_eraser);
	}
    }
    
    /// And now call for region growing on each cluster from laser
    marker_id=0;
    
    for (vector<PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it)
    {
	
	marker_id++;
	typename PointCloud<T1>::Ptr cluster (new PointCloud<T1>);
	typename PointCloud<T>::Ptr cluster_growed (new PointCloud<T>);
	cluster->header.frame_id=msg->header.frame_id;
	cluster_growed->header.frame_id=msg->header.frame_id;
	for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cluster->points.push_back (laser_scan_ptr->points[*pit]);   
	
// 	visualization_msgs::Marker marker=mark_cluster(cluster, "Obstacle" ,marker_id, 1.0, 0.0, 0.0);
// 	pub_marker_ptr->publish(marker);
	
	    if ((int)cluster->points.size()>0)
	    {
		PointIndices::Ptr region_indices (new PointIndices ());
		
		region_growing<T, T1> rg;
		rg.radius=radius;
		rg.set_input_seeds(cluster);
		rg.set_input_cloud(cloud);
		
		rg.filter();
		
		*region_indices=rg.get_region_point_indices();
		
		if (region_indices->indices.size()>0)
		{
		    // Extract the inliers
		    ExtractIndices<T> extract;
		    extract.setInputCloud (cloud);
		    
		    extract.setIndices (region_indices);
		    extract.setNegative (false);
		    extract.filter (*cluster_growed);
		    
		    visualization_msgs::Marker marker_info=mk.mark_text(cluster_growed, "Obstacle" ,marker_id, 0.0, 1.0, 0.0);
		    visualization_msgs::Marker marker=mk.mark(cluster_growed, "Obstacle" ,marker_id, 1.0, 0.0, 0.0);
		    pub_markers_ptr->publish(marker);
		    pub_markers_text_ptr->publish(marker_info);
		}
		cluster_growed.reset();
		region_indices.reset();
	    }
    }
        
    cloud.reset();
    laser_scan_ptr.reset();
};
//////////////////////////////////////////////////////////////////////////////////////////////

template<class T, class T1>
void region_growing_node<T,T1>::callback_laser (const sensor_msgs::PointCloud2Ptr& msg)
{
//     fromROSMsg(*msg,laser_scan);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, laser_scan);
};
//////////////////////////////////////////////////////////////////////////////////////////////

#endif
