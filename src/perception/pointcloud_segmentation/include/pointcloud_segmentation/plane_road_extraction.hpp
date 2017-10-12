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
 * \brief plane_road_extraction class implementation
 */

#ifndef _PLANE_ROAD_EXTRACTION_HPP_
#define _PLANE_ROAD_EXTRACTION_HPP_

#include "plane_road_extraction.h"

template <class T>
void plane_road_extraction<T>::filter (const sensor_msgs::PointCloud2Ptr& msg)
{
    PointCloud<T> cloud;
    
    fromROSMsg(*msg,cloud);
    
    if ((int)cloud.points.size()>0)
    {
	tf::Transform road_tf=pms.find_and_publish_road_frame(cloud);    
	
	//   Create the convex hull to extract the point clouds
	T point_origin;
	T point_origin_transformed;
	T point_away;
	T point_away_transformed;
	T pt;
	
	point_origin.x=0; point_origin.y=0; point_origin.z=0;                               // Transform a point on the origin
	point_away.x=50; point_away.y=0; point_away.z=0;                                    // Transform a point far (10m)
	point_origin_transformed=transform_pcl_point(cloud.header.frame_id, road_tf, point_origin);
	point_away_transformed=transform_pcl_point(cloud.header.frame_id, road_tf, point_away);
	
	PointCloud<T> road_hull;
	road_hull.header.frame_id=cloud.header.frame_id;
	pt.x = -2; pt.y=50; pt.z=point_origin_transformed.z;
	road_hull.points.push_back(pt);
	pt.x = 50; pt.y=50; pt.z=point_away_transformed.z;
	road_hull.points.push_back(pt);
	pt.x = 50; pt.y= -50; pt.z=point_away_transformed.z;
	road_hull.points.push_back(pt);
	pt.x = -2; pt.y=-50; pt.z=point_origin_transformed.z;
	road_hull.points.push_back(pt);
	
	// Roadless point cloud
	PointCloud<T> cloud_road;
	PointCloud<T> cloud_cluster;
	points_from_volume<T> pfv_cloud;
	cloud_road.header.frame_id=cloud.header.frame_id;
	cloud_cluster.header.frame_id=cloud.header.frame_id;
	pfv_cloud.set_convex_hull(road_hull);
	pfv_cloud.convexhull_function(cloud, z_max, z_min, false);
	cloud_road=pfv_cloud.get_pc_in_volume();
	pfv_cloud.convexhull_function(cloud, 20, z_max, false);
	cloud_cluster=pfv_cloud.get_pc_in_volume();
	
	//   Project road cloud on the plane
	PointCloud<T> cloud_road_projected; 
	cloud_road_projected = project_cloud_on_plane(cloud_road, pms.coef);
	
	// Calculate the road perimeter
	PointCloud<T> road_perimeter; 
	ConvexHull<T> chull;
	chull.setInputCloud (cloud_road_projected.makeShared());
	chull.reconstruct (road_perimeter);
	
	// Create an extraction object
	points_from_volume<T> pfv;
	PointCloud<T> inside_cloud; 
	inside_cloud.header.frame_id=msg->header.frame_id;
	pfv.set_convex_hull(road_perimeter);
	pfv.convexhull_function(cloud_cluster, 20, -20, false);
	inside_cloud=pfv.get_pc_in_volume();                //  ->  All points inside road convex_hull
	
	// 	cout << cloud_cluster.points.size() << endl;
	
	sensor_msgs::PointCloud2 msg_road;
	sensor_msgs::PointCloud2 msg_cluster;
	sensor_msgs::PointCloud2 msg_perimeter;
	toROSMsg(cloud_road_projected,msg_road);
	toROSMsg(inside_cloud,msg_cluster);
	toROSMsg(road_perimeter,msg_perimeter);
	pub_clusters_ptr->publish(msg_cluster);
	pub_road_ptr->publish(msg_road);
	pub_road_perimeter_ptr->publish(msg_perimeter);
	
    }
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
T plane_road_extraction<T>::transform_pcl_point(string frame1, tf::Transform transform, T pt_origin)
{
    PointCloud<T> pc_point;   // A point cloud with a single point
    pc_point.points.push_back(pt_origin);
    pcl_ros::transformPointCloud (pc_point,pc_point,transform);
    return pc_point.points[0];
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
PointCloud<T> plane_road_extraction<T>::project_cloud_on_plane(PointCloud<T> cloud_in, ModelCoefficients::Ptr coefficients)
{
    typename PointCloud<T>::Ptr cloud_in_ptr (new PointCloud<T> (cloud_in));
    PointCloud<T> cloud_out;
    
    //Create the projection object
    ProjectInliers<T> projection;
    projection.setModelType(SACMODEL_PLANE); //set model type
    
    projection.setInputCloud(cloud_in_ptr);
    projection.setModelCoefficients(coefficients);
    projection.filter(cloud_out);
    
    cloud_in_ptr.reset();
    return cloud_out;
}

#endif
