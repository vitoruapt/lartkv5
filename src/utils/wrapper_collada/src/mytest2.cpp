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
 * @file
 * @brief A simple test file. Second try. Don't remember much else.
 */ 


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <wrapper_collada/wrapper_collada.h>
#define  PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}




int main(int argc, char* argv[]) {


	//daeElement* bindVertexInput = instanceMaterial->add("bind_vertex_input");
	//bindVertexInput->setAttribute("semantic", "uv0");
	//bindVertexInput->setAttribute("input_semantic", "TEXCOORD");
	//bindVertexInput->setAttribute("input_set", "0");



	//MAIN CYCLE
	ros::init(argc, argv, "point_cloud_publisher"); // Initialize ROS coms                  
	ros::NodeHandle n; //The node handle
	ros::Rate r(0.1);
	visualization_msgs::Marker marker;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/wrapper_collada", 1);
	marker.id = 1;	
	marker.header.stamp = ros::Time::now();	
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;	
	marker.action = visualization_msgs::Marker::ADD;	
	marker.ns = "ns";
	//marker.mesh_resource = "package://wrapper_collada/bin/polygons1.dae";	
	marker.mesh_use_embedded_materials = 1;	
	marker.header.frame_id = "/atc/vehicle/ground";
	//marker.scale.x = 2.54/100;
	//marker.scale.y = 2.54/100;
	//marker.scale.z = 2.54/100;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.r = 0.2;
	marker.color.g = 0.3;
	marker.color.b = 0.4;
	marker.color.a = 1;
int a=0;
 while(n.ok())
{
	a++;
	//char str[1024];
	//sprintf(str, "mesh/polygons%d.dae",a);
	//wrapper_collada wc(str);

	////Emulating a function that produces a dae given a polygon
	////The input arguments will be
	//std::string polygon_name("plg0");//the polygon name	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;	//the point cloud with the polygon vertices list
	//cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud->points.push_back(pcl::PointXYZ(-1,-1, 0));
	//cloud->points.push_back(pcl::PointXYZ(1,-1, 0));
	//cloud->points.push_back(pcl::PointXYZ(1,1, 0));
	//cloud->points.push_back(pcl::PointXYZ(-1,1, 0));

	//pcl::PointCloud<pcl::PointXYZ>::Ptr normal;	//the point cloud with the polygon vertices list
	//normal = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//normal->points.push_back(pcl::PointXYZ(0, 0, 1)); //the normal vector to the polygon


	//wc.add_polygon_fixed_color(polygon_name, cloud, normal, 1,0,0,1); 

	//wc.write_file();	

	char str1[1024];
	//sprintf(str1, "package://wrapper_collada/bin/%s",str);
	//sprintf(str1, "package://wrapper_collada/models/car1.dae");
	sprintf(str1, "package://wrapper_collada/models/decisive_woman.obj");
	marker.mesh_resource = str1;
	marker.header.stamp = ros::Time::now();	
	marker_pub.publish(marker);

	ros::Duration(0.4).sleep();
	ros::spinOnce();

}

return 0;
}
