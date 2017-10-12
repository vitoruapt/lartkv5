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
#include <navigability_map/navigability_map.h>
#include "representation_rviz.h"
#include <pcl_conversions/pcl_conversions.h>

/**
  \brief Nodelet that use Navigability_Map class to generate a Accessibility Map.
 * This nodelet subscribes sensor_msgs::PointCloud2 and filters the PointCloud, 
 * then it generates a Grid with each node containing data
 * from the PointCloud
 \file main.cpp
 \author Diogo Matos
 \date June 2013
 **/


//GLOBAL VARIABLES
Navigability_Map *g_NavMap;
bool cloud_arrived=false;

ros::Publisher elevation_map_pub;
ros::Publisher normals_pub;
ros::Publisher accessibility_map_pub;
ros::Publisher polygon_pub;
ros::Publisher obstacle_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filter (new pcl::PointCloud<pcl::PointXYZ>);

tf::TransformListener *p_listener;

/**
 * \brief Callback from the Pointcloud subscribed topic
 * This calback filter the pointcloud uses de Navigability_Map class to generate a navigability_map
 * \param[in] pcmsg_in PointCloud subscribed
 */

void pointcloud_cb (sensor_msgs::PointCloud2 pcmsg_in)
{
	visualization_msgs::MarkerArray grid_markers_elavationmap, accessibility_markers;
	visualization_msgs::Marker marker_normals;
	visualization_msgs::Marker marker_polygon;
	visualization_msgs::Marker marker_obstacle;
	
	//Cloud received
	pcl::PointCloud<pcl::PointXYZ> pc_in;
	pcl::fromROSMsg(pcmsg_in,pc_in);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(pcmsg_in, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pc_in);
    
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_in=pc_in;
	
	//filter cloud
	g_NavMap->Filter_PointCloud(cloud_in,pc_filter);
	
	//Normal Estimation
	std::cout<<"Normal Estimation"<<std::endl;
	g_NavMap->Normal_Estimation(pc_filter);

	double max_Xval=g_NavMap->Xmax_filter; 									//X max range value
	double max_Yval=max(abs(g_NavMap->Ymin_filter),g_NavMap->Ymax_filter);	//y max range value
 
	//seting the grid parameter
	g_NavMap->setGrid_parameter(max_Xval,max_Yval); 
	
	//inicialize grid
	g_NavMap->inicialize_grid();

	//seting grid data
	g_NavMap->setGrid_data(*pc_filter);

	//calc grid data
	g_NavMap->calcGrid_data();

	//fill empty cells
	g_NavMap->fill_data_cells();
	
	//calc Cells_accessibility
	g_NavMap->set_Cells_accessibility();
 	
	//_______________
	//               |
	//   markers     |
	//_______________|
	
	//elevation map;
	grid_markers_elavationmap.markers=create_Markers_ElavationMap(*g_NavMap, pc_filter->header.frame_id );
	//normals
	marker_normals=create_Markers_Normals(*g_NavMap, pc_filter->header.frame_id);
	//accessibility
	accessibility_markers.markers=create_Markers_AccessibilityMap(*g_NavMap, pc_filter->header.frame_id);
	
// 	publish
	elevation_map_pub.publish(grid_markers_elavationmap);
	normals_pub.publish(marker_normals);
	accessibility_map_pub.publish(accessibility_markers);
	
	
	//_______________
	//               |
	//  Ground truth |
	//_______________|
// 		g_NavMap->polygon_groundtruth();
// 		g_NavMap->getCell_inpolygon();
// 		g_NavMap->dataCell_inpolygon();
// 	
// 		marker_polygon=create_Markers_Polygon(*g_NavMap, pc_filter->header.frame_id);
// 		marker_obstacle=create_Markers_Obstacle(*g_NavMap, pc_filter->header.frame_id);
// 		polygon_pub.publish(marker_polygon);
// 		obstacle_pub.publish(marker_obstacle);	
	
		
	
	cloud_arrived=true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigability_map");
	ros::NodeHandle n("~");

	tf::TransformListener listener(n,ros::Duration(1000));
	p_listener=&listener;
	
// 	Declare the class
	Navigability_Map navmap;
	g_NavMap=&navmap;
	
	double output_freq;
	
	//Seting Param
	n.param("output_frequency", output_freq, 200.0);
	n.param("PointCLoud_Xmin_filter",g_NavMap->Xmin_filter,0.0);
	n.param("PointCLoud_Xmax_filter",g_NavMap->Xmax_filter,22.5);
	n.param("PointCLoud_Ymax_filter",g_NavMap->Ymax_filter,22.5);
	n.param("PointCLoud_Ymin_filter",g_NavMap->Ymin_filter,-22.5);
	n.param("PointCLoud_Zmin_filter",g_NavMap->Zmin_filter,-15.0);
	n.param("PointCLoud_Zmax_filter",g_NavMap->Zmax_filter,2.0);
	n.param("Grid_Sx",g_NavMap->Sx,0.2);
	n.param("Grid_Sy",g_NavMap->Sy,0.2);
	n.param("Radius_neighbors",g_NavMap->Radius_neighbors,0.3);
	n.param("K_neighbors",g_NavMap->K_neighbors,10);
	n.param("Use_Radius_Search",g_NavMap->Use_Radius_Search,1);
	n.param("Zmax_heigh_difference",g_NavMap->Zmax_heigh_difference,0.1);
	n.param("angleX_max_difference",g_NavMap->angleX_max_difference,0.1);
	n.param("angleY_max_difference",g_NavMap->angleY_max_difference,0.1);
	n.param("angleZ_max_difference",g_NavMap->angleZ_max_difference,0.1);
	n.param("debug_accessibility",g_NavMap->debug_accessibility,1);
	n.param("default_confidence",g_NavMap->default_confidence,0.5);
	n.param("fator_confidence_neighbour_limit",g_NavMap->fator_confidence_neighbour_limit,1.5);
	n.param("Standard_Deviation_max_confidence",g_NavMap->Standard_Deviation_max,0.2);
	n.param("Standard_Deviation_anglex_max_confidence",g_NavMap->Standard_Deviation_anglex_max_confidence,0.2);
	n.param("Standard_Deviation_angley_max_confidence",g_NavMap->Standard_Deviation_angley_max_confidence,0.2);
	n.param("Standard_Deviation_anglez_max_confidence",g_NavMap->Standard_Deviation_anglez_max_confidence,0.2);

	// 	declare the subsriber
	ros::Subscriber sub_phiversion = n.subscribe ("/pointcloud0", 1, pointcloud_cb);

// 	declare advertiser
	//normals
	normals_pub = n.advertise<visualization_msgs::Marker>("/normals", 1);
	//pointcloud filter
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/pc_filter_navmap", 1);	
	//maps
	elevation_map_pub = n.advertise<visualization_msgs::MarkerArray>("/elevation_map", 1);
	accessibility_map_pub = n.advertise<visualization_msgs::MarkerArray>("/accessibility_map", 1);

	//ground truth
	polygon_pub = n.advertise<visualization_msgs::Marker>("/polygon", 1);
	obstacle_pub = n.advertise<visualization_msgs::Marker>("/obstacle", 1);

	ros::Rate loop_rate(output_freq);

	while (n.ok())
	{
		ros::spinOnce();

		if (cloud_arrived)
		{
			sensor_msgs::PointCloud2 pcmsg_out;
			pcl::toROSMsg(*pc_filter, pcmsg_out);
			pcmsg_out.header.stamp = ros::Time::now();
			//publish the pointcloud
			pub.publish(pcmsg_out);
			
			cloud_arrived=false;
		}
		loop_rate.sleep();	
	}
}
