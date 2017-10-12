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
/** \brief A nodelet to implement the euclidean_cluster
 *  \file
 *  \author Tiago Talhada
 *  \date June 2012
 */

#include <pointcloud_segmentation/euclidean_cluster.h>

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "euclidean_cluster_nodelet");
    ros::NodeHandle n;
    
    // Variables to read from launch file
    string pointcloud_input;
    string cluster_markers;
    string cluster_markers_text;
    
    // Create the euclidean_cluster_class object
    euclidean_cluster_extraction<PointXYZRGB> * ec_class = new euclidean_cluster_extraction<PointXYZRGB> ();
    
    // Read data from launch file
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pointcloud_input", pointcloud_input, string("pointcloud_input"));
    private_node_handle_.param("cluster_markers", cluster_markers, string("cluster_markers"));
    private_node_handle_.param("cluster_markers_text", cluster_markers_text, string("cluster_markers_text"));
    private_node_handle_.param("cluster_tolerance", ec_class->cluster_tolerance, double(ec_class->cluster_tolerance));
    private_node_handle_.param("min_cluster_size", ec_class->min_cluster_size, double(ec_class->min_cluster_size));
    private_node_handle_.param("max_cluster_size", ec_class->max_cluster_size, double(ec_class->max_cluster_size));
    
    // Subscribe messages
    ros::Subscriber sub_cloud = n.subscribe(pointcloud_input.c_str(), 1, &euclidean_cluster_extraction<PointXYZRGB>::callback_cloud,ec_class);
    
    // Publish messages
    ros::Publisher pub_markers=n.advertise<visualization_msgs::Marker>(cluster_markers.c_str(),1000);
    ros::Publisher pub_markers_text=n.advertise<visualization_msgs::Marker>(cluster_markers_text.c_str(),1000);
    
    // Set object pointers
    ec_class->pub_marker_ptr=&pub_markers;
    ec_class->pub_marker_text_ptr=&pub_markers_text;
    
    // Main loop.
    while (n.ok())
    {
	ros::spinOnce();
    }
    
    return 0;
}
