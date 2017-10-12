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
 * \brief euclidean_cluster_extraction class implementation
 */

#ifndef _EUCLIDEAN_CLUSTER_HPP_
#define _EUCLIDEAN_CLUSTER_HPP_


#include <pointcloud_segmentation/euclidean_cluster.h>

template<class T>
void euclidean_cluster_extraction<T>::callback_cloud (const sensor_msgs::PointCloud2Ptr& msg)
{
    typename PointCloud<T>::Ptr cloud (new PointCloud<T> ());
    fromROSMsg(*msg,*cloud);
    if ((int)cloud->points.size()>0)
    {
	// Euclidean segmentation
	    EuclideanClusterExtraction<T> ec_inside;
	    vector<PointIndices> inside_indices;
	    ec_inside.setClusterTolerance (cluster_tolerance); // 20cm
	    ec_inside.setMinClusterSize (min_cluster_size);
	    ec_inside.setMaxClusterSize (max_cluster_size);
	    ec_inside.setInputCloud (cloud);
	    ec_inside.extract (inside_indices);
	    
	    // Declare markers class
	    markers<T> mk;
	    
	    // Delete all previous markers
	    static int marker_id;
	    if (marker_id>0)
	    {
		for (int i=0; i<=marker_id; ++i)
		{
		    visualization_msgs::Marker marker_eraser=mk.unmark(msg->header.frame_id, "Obstacle", i);
		    pub_marker_ptr->publish(marker_eraser);
		    pub_marker_text_ptr->publish(marker_eraser);
		}
	    }
	    
	    marker_id=0;
	    for (vector<PointIndices>::const_iterator it = inside_indices.begin (); it != inside_indices.end (); ++it)
	    {
		typename PointCloud<T>::Ptr inside_cluster (new PointCloud<T>);
		inside_cluster->header.frame_id=msg->header.frame_id;
		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		    inside_cluster->points.push_back (cloud->points[*pit]);   
		
		visualization_msgs::Marker marker_info=mk.mark_text(inside_cluster, "Obstacle" ,marker_id, 1.0, 0.0, 0.0);
		visualization_msgs::Marker marker=mk.mark(inside_cluster, "Obstacle" ,marker_id, 0.0, 1.0, 0.0);
		marker_id++;
		pub_marker_ptr->publish(marker);
		pub_marker_text_ptr->publish(marker_info);
		inside_cluster.reset();
	    }
	    
    }
    else
	ROS_ERROR("EUCLIDEAN_CLUSTERING: NO DATA POINTS RECEIVED!");
   
    cloud.reset();
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
