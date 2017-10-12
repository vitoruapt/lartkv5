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
/** \brief A nodelet to implement and extract the road 
 *  \file
 *  \author Tiago Talhada
 *  \date June 2012
 */

#include <pointcloud_segmentation/plane_road_extraction.h>

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "plane_road_extraction_nodelet");
    ros::NodeHandle n;
    
    // Variables from launch file
    string pointcloud_input;
    string pointcloud_road;
    string pointcloud_clusters;
    string pointcloud_road_perimeter;
    double threshold;
    
    // Declare the node class object
    plane_road_extraction<PointXYZRGB>  * extraction = new plane_road_extraction<PointXYZRGB>();
    
    // Read parameters from launch file
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pointcloud_input", pointcloud_input, string("pointcloud_input"));
    private_node_handle_.param("pointcloud_road", pointcloud_road, string("pointcloud_road"));
    private_node_handle_.param("pointcloud_clusters", pointcloud_clusters, string("pointcloud_clusters"));
    private_node_handle_.param("pointcloud_road_perimeter", pointcloud_road_perimeter, string("pointcloud_road_perimeter"));
    private_node_handle_.param("z_min", extraction->z_min, double(extraction->z_min));
    private_node_handle_.param("z_max", extraction->z_max, double(extraction->z_max));
    private_node_handle_.param("threshold", threshold, double(threshold));
    
    // Subscribe a pointcloud message
    ros::Subscriber sub_cloud = n.subscribe(pointcloud_input.c_str(), 1000, &plane_road_extraction<PointXYZRGB>::filter, extraction);
    
    // Publish a pointcloud message
    ros::Publisher pub_road = n.advertise<sensor_msgs::PointCloud2>(pointcloud_road.c_str(), 10);
    ros::Publisher pub_clusters = n.advertise<sensor_msgs::PointCloud2>(pointcloud_clusters.c_str(), 10);
    ros::Publisher pub_road_perimeter= n.advertise<sensor_msgs::PointCloud2>(pointcloud_road_perimeter.c_str(), 10);
    
    // Set pointers
    extraction->pub_road_ptr=&pub_road;
    extraction->pub_clusters_ptr=&pub_clusters;
    extraction->pub_road_perimeter_ptr=&pub_road_perimeter;
    
    // Configure tf 
    tf::TransformListener listener;      // atc/vehicle/ground
    tf::TransformBroadcaster br;
    
    extraction->pms.listener_atc_ground_ptr=&listener;
    extraction->pms.broadcast_ptr=&br;

    // Configure your frame names
    extraction->pms.reference_ground_frame=(char *)(ros::names::remap("/atc/vehicle/ground")).c_str();
    extraction->pms.pub_frame_name=(char *)(ros::names::remap("/environment/road_frame")).c_str(); 
    extraction->pms.threshold=(float)(threshold);
    extraction->pms.debug=false;
    
    // Main loop.
    while (n.ok())
    {
	ros::spinOnce();
    }
    
    return 0;
}
