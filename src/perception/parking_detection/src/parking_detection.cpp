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
 * @brief Location of a parking spot
 * @details From the point cloud received, a parking spot with certain spec is searched. After that, a marker change his colour.
 * @author Joel
 * @date 5-May-2012
 */

#include <stdio.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <points_from_volume/points_from_volume.h>
#include <trajectory_planner/coordinates.h>

// Defines
#define PFLN printf("FILE %s LINE %d\n",__FILE__, __LINE__);

// Namespaces
using namespace std;

// Global vars
ros::NodeHandle* p_n;
ros::Publisher cloud_pub, cloud_pub2, cloud_pub3, cloud_pub4, Publisher, cloud_pub5;
tf::TransformListener *p_listener;
pcl::PointCloud<pcl::PointXYZ> convex_hull1, convex_hull2, convex_hull3;
ros::Publisher vis_pub, vis_pub2, vis_pub3, vis_pub4, car_pub;
points_from_volume<pcl::PointXYZ> pfv, pfv2, pfv3, pfv4;
trajectory_planner::coordinates message;

double spot_length=1.5;         // Length of vehicle (parking spot) was 1.2
double spot_wide=0.45;          // Wide
double spot_high=0.6;           // High
double spot_distance=0.125;     // distance from (between) vehicles    -> this must be bigger (0.225 i guess)
double SpotDetected=0;

/** 
 * @brief Publishes filtered pcl & extruded pcl
 * @param PointCloud message received
 * @return void
 */
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr & pcmsg_in)
{
	// STEP 1: Create the point_cloud input
	pcl::PointCloud<pcl::PointXYZ> pc_in;
	pcl::fromROSMsg(*pcmsg_in,pc_in);

	
	//STEP 2: Query for the transwformation to use
   	tf::StampedTransform transform;
	try
	{
		p_listener->lookupTransform(pcmsg_in->header.frame_id, "/vehicle_odometry", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}


	//STEP3: transform pc_in using the queried transform
	pcl::PointCloud<pcl::PointXYZ> pc_transformed;
	pcl::PointCloud<pcl::PointXYZ> pc_ahead, pc_spot, pc_behind, pc_ground;
	pcl_ros::transformPointCloud(pc_in,pc_transformed, transform.inverse());
    pc_transformed.header.frame_id = "/vehicle_odometry";
	
	
	// STEP 4: Aplying the ConvexHull class
	pfv.convexhull_function(pc_transformed, 0.0, -0.6, false);
	pc_ahead=pfv.get_pc_in_volume();
	pc_ahead.header.frame_id = "/vehicle_odometry";
	
	pfv2.convexhull_function(pc_transformed, 0.0, -0.6, false);
	pc_spot=pfv2.get_pc_in_volume();
	pc_spot.header.frame_id = "/vehicle_odometry";
	
	pfv3.convexhull_function(pc_transformed, 0.0, -0.6, false);
	pc_behind=pfv3.get_pc_in_volume();
	pc_behind.header.frame_id = "/vehicle_odometry";
	
	pfv4.convexhull_function(pc_transformed, 0.07, 0.0, false);
	pc_ground=pfv4.get_pc_in_volume();
	pc_ground.header.frame_id = "/vehicle_odometry";
	
	
	// STEP 5: Convert to ROSMsg
	sensor_msgs::PointCloud2 pcmsg_out;
	pcl::toROSMsg(pc_ahead, pcmsg_out);
	
	sensor_msgs::PointCloud2 pcmsg_out2;
	pcl::toROSMsg(pc_spot, pcmsg_out2);

	sensor_msgs::PointCloud2 pcmsg_out3;
	pcl::toROSMsg(pc_behind, pcmsg_out3);
	
	sensor_msgs::PointCloud2 pcmsg_out4;
	pcl::toROSMsg(pc_ground, pcmsg_out4);
	
	
	// STEP 6: Markers 
	// Marker car
	visualization_msgs::Marker marker_car;
	marker_car.header.frame_id = "/vehicle_odometry";                // Frame name
	marker_car.header.stamp = ros::Time();
	marker_car.ns = "my_namespace";
	marker_car.id = 0;
	marker_car.type = visualization_msgs::Marker::CUBE;       // Marker type
	marker_car.action = visualization_msgs::Marker::ADD;
	marker_car.scale.x = 0.8;
	marker_car.scale.y = spot_wide;
	marker_car.scale.z = spot_high;
	marker_car.pose.position.x = 0.8/2;                             // Position
	marker_car.pose.position.y = 0;
	marker_car.pose.position.z = spot_high/2;
	marker_car.pose.orientation.x = 0.0;                        // Orientation related to the frame
	marker_car.pose.orientation.y = 0.0;
	marker_car.pose.orientation.z = 0.0;
	marker_car.pose.orientation.w = 1.0;
	marker_car.color.a = 0.2;
	marker_car.color.r = 0.0;
	marker_car.color.g = 0.0;
	marker_car.color.b = 1.0;
	
	int condition=0;
	// Marker 1
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/vehicle_odometry";                // Frame name
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;       // Marker type
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = spot_length;
	marker.scale.y = spot_wide;
	marker.scale.z = spot_high;
	marker.pose.position.x = spot_length/2;                             // Position
	marker.pose.position.y = spot_wide + spot_distance;
	marker.pose.position.z = spot_high/2;
	marker.pose.orientation.x = 0.0;                        // Orientation related to the frame
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.color.a = 0.3;
	if(pc_ahead.points.size()<5)
	{
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		condition=condition+0.5;
	}
	else
	{
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}
	
	// Marker 2
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "/vehicle_odometry";                // Frame name
	marker2.header.stamp = ros::Time();
	marker2.ns = "my_namespace";
	marker2.id = 0;
	marker2.type = visualization_msgs::Marker::CUBE;       // Marker type
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.scale.x = spot_length;
	marker2.scale.y = spot_wide;
	marker2.scale.z = spot_high;
	marker2.pose.position.x = -spot_length/2;                             // Position
	marker2.pose.position.y = spot_wide + spot_distance;
	marker2.pose.position.z = spot_high/2;
	marker2.pose.orientation.x = 0.0;                        // Orientation related to the frame
	marker2.pose.orientation.y = 0.0;
	marker2.pose.orientation.z = 0.0;
	marker2.pose.orientation.w = 1.0;
	marker2.color.a = 0.3;
	if(pc_spot.points.size()<5)
	{
		marker2.color.r = 0.0;
		marker2.color.g = 1.0;
		marker2.color.b = 0.0;
		condition=condition+1;
	}
	else
	{
		marker2.color.r = 1.0;
		marker2.color.g = 0.0;
		marker2.color.b = 0.0;
	}
	
	// Marker 3
	visualization_msgs::Marker marker3;
	marker3.header.frame_id = "/vehicle_odometry";                // Frame name
	marker3.header.stamp = ros::Time();
	marker3.ns = "my_namespace";
	marker3.id = 0;
	marker3.type = visualization_msgs::Marker::CUBE;       // Marker type
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.scale.x = spot_length;
	marker3.scale.y = spot_wide;
	marker3.scale.z = spot_high;
	marker3.pose.position.x = -spot_length - spot_length/2;                             // Position
	marker3.pose.position.y = spot_wide + spot_distance;
	marker3.pose.position.z = spot_high/2;
	marker3.pose.orientation.x = 0.0;                        // Orientation related to the frame
	marker3.pose.orientation.y = 0.0;
	marker3.pose.orientation.z = 0.0;
	marker3.pose.orientation.w = 1.0;
	marker3.color.a = 0.3;
	if(pc_behind.points.size()<5)
	{
		marker3.color.r = 1.0;
		marker3.color.g = 0.0;
		marker3.color.b = 0.0;
	}
	else
	{
		marker3.color.r = 0.0;
		marker3.color.g = 1.0;
		marker3.color.b = 0.0;
		condition=condition+1; 
	}
	
	// Marker 4
	visualization_msgs::Marker marker4;
	marker4.header.frame_id = "/vehicle_odometry";                // Frame name
	marker4.header.stamp = ros::Time();
	marker4.ns = "my_namespace";
	marker4.id = 0;
	marker4.type = visualization_msgs::Marker::CUBE;       // Marker type
	marker4.action = visualization_msgs::Marker::ADD;
	marker4.scale.x = spot_length;
	marker4.scale.y = spot_wide;
	marker4.scale.z = 0.05;
	marker4.pose.position.x = - spot_length/2;                             // Position
	marker4.pose.position.y = spot_wide + spot_distance;
	marker4.pose.position.z = -0.025;
	marker4.pose.orientation.x = 0.0;                        // Orientation related to the frame
	marker4.pose.orientation.y = 0.0;
	marker4.pose.orientation.z = 0.0;
	marker4.pose.orientation.w = 1.0;
	marker4.color.a = 0.6;
	marker4.color.r = 0.4;
	marker4.color.g = 0.8;
	marker4.color.b = 0.9;	
	if(pc_ground.points.size()<5)
	{
		marker4.color.r = 1.0;
		marker4.color.g = 0.0;
		marker4.color.b = 0.0;
	}
	else
	{
		marker4.color.r = 0.0;
		marker4.color.g = 1.0;
		marker4.color.b = 0.0;
		condition=condition+1;
	}


	// STEP 7: Publish markers and PClouds
	car_pub.publish(marker_car);
	vis_pub.publish(marker);
	vis_pub2.publish(marker2);
	vis_pub3.publish(marker3);
	vis_pub4.publish(marker4);
	
    cloud_pub.publish(pcmsg_out);
	cloud_pub2.publish(pcmsg_out2);
	cloud_pub3.publish(pcmsg_out3);
	cloud_pub4.publish(pcmsg_out4);
	
	
	// STEP 8: Send empty space coordinates
	double xpos=marker2.pose.position.x;                 
	double ypos=marker2.pose.position.y;
	double zpos=0.0;
	
	tf::StampedTransform transform2;
			
	try
	{
		p_listener->lookupTransform("/vehicle_odometry", "/world", ros::Time(0), transform2);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
// 		return 0;
	}
	
	pcl::PointCloud<pcl::PointXYZ> pc_spot_coordinate_in;
	pcl::PointCloud<pcl::PointXYZ> pc_spot_coordinate_out;
	
	pcl::PointXYZ pt1;
	pt1.x = xpos-0.50; 
	pt1.y= ypos+0.14;//cheat 
	pt1.z= zpos;
	pc_spot_coordinate_in.points.push_back(pt1);
	
	pcl_ros::transformPointCloud(pc_spot_coordinate_in,pc_spot_coordinate_out, transform2.inverse());
    pc_spot_coordinate_out.header.frame_id = "/world";
	
	double orix;
	double oriy;
	double oriz; 
	double oriw;
	
	orix=transform2.getRotation()[0];
	oriy=transform2.getRotation()[1];
	oriz=transform2.getRotation()[2];
	oriw=transform2.getRotation()[3];
	
	double thheta=atan2(2*(oriw*oriz+orix*oriy),1-2*(oriy*oriy+oriz*oriz));
// 	cout<<"CENASS "<<orix<<" "<<oriy<<" "<<oriz<<" "<<oriw<<" "<<endl;
// 	cout<<"CENASS "<<thheta<<endl;
	
	// ______________________
	//|                      |
	//|  Parking condition   |
	//|______________________|		
	if(condition>=3 && SpotDetected==0)
	{
		
		// Send message
		message.x=(pc_spot_coordinate_out.points[0].x);//-0.3*cos(thehta);
		message.y=(pc_spot_coordinate_out.points[0].y);//-0.3*sin(thheta);
		message.theta=-thheta;
		
		printf("Parking in: %f %f \n",pc_spot_coordinate_out.points[0].x,pc_spot_coordinate_out.points[0].y);
		
		sensor_msgs::PointCloud2 pcmsg_parking_place;
		pcl::toROSMsg(pc_spot_coordinate_out, pcmsg_parking_place);
		cloud_pub5.publish(pcmsg_parking_place);
		
		
		
		
		
		Publisher.publish(message);
		SpotDetected=1;

	}
}

/** 
 * @brief Main function of the parking detection node
 * @param int argc
 * @param char **argv
 * @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_transformer_node");                             
	ros::NodeHandle n;
	tf::TransformListener listener(n,ros::Duration(10));
	p_listener=&listener;
	p_n=&n;
	Publisher = n.advertise<trajectory_planner::coordinates>("/msg_coordinates", 1000);
	
	// ______________________
	//|                      |
	//|       Markers        |
	//|______________________|
	car_pub = n.advertise<visualization_msgs::Marker>( "car", 0 );
	vis_pub = n.advertise<visualization_msgs::Marker>( "parking_ahead", 0 );
 	vis_pub2 = n.advertise<visualization_msgs::Marker>( "parking_spot", 0 );
	vis_pub3 = n.advertise<visualization_msgs::Marker>( "parking_behind", 0 );
	vis_pub4 = n.advertise<visualization_msgs::Marker>( "ground", 0 );
	
	// ______________________
	//|                      |
	//|     ConvexHulls      |
	//|______________________|
	// ConvexHull 1
	convex_hull1.header.frame_id="/vehicle_odometry";
	pcl::PointXYZ pt1;
	pt1.x = spot_length/2 + spot_length/2; pt1.y= (spot_wide + spot_distance) + spot_wide/2; pt1.z= 0.02;
	convex_hull1.points.push_back(pt1);
	pt1.x = spot_length/2 + spot_length/2; pt1.y= (spot_wide + spot_distance) - spot_wide/2; pt1.z= 0.02;
	convex_hull1.points.push_back(pt1);
	pt1.x = spot_length/2 - spot_length/2; pt1.y= (spot_wide + spot_distance) - spot_wide/2; pt1.z= 0.02;
	convex_hull1.points.push_back(pt1);
	pt1.x = spot_length/2 - spot_length/2; pt1.y= (spot_wide + spot_distance) + spot_wide/2; pt1.z= 0.02;
	convex_hull1.points.push_back(pt1);
	pfv.set_convex_hull(convex_hull1);

	// ConvexHull 2
	convex_hull2.header.frame_id="/vehicle_odometry";
	pcl::PointXYZ pt2;
	pt2.x = spot_length/2 - spot_length/2; pt2.y= (spot_wide + spot_distance) + spot_wide/2; pt2.z= 0.02;
	convex_hull2.points.push_back(pt2);
	pt2.x = spot_length/2 - spot_length/2; pt2.y= (spot_wide + spot_distance) - spot_wide/2; pt2.z= 0.02;
	convex_hull2.points.push_back(pt2);
	pt2.x = - spot_length; pt2.y= (spot_wide + spot_distance) - spot_wide/2; pt2.z= 0.02;
	convex_hull2.points.push_back(pt2);
	pt2.x = - spot_length; pt2.y= (spot_wide + spot_distance) + spot_wide/2; pt2.z= 0.02;
	convex_hull2.points.push_back(pt2);
	pfv2.set_convex_hull(convex_hull2);
	
	// ConvexHull 3
	convex_hull3.header.frame_id="/vehicle_odometry";
	pcl::PointXYZ pt3;
	pt3.x = - spot_length; pt3.y= (spot_wide + spot_distance) + spot_wide/2; pt3.z= 0.02;
	convex_hull3.points.push_back(pt3);
	pt3.x = - spot_length; pt3.y= (spot_wide + spot_distance) - spot_wide/2; pt3.z= 0.02;
	convex_hull3.points.push_back(pt3);
	pt3.x = - spot_length - spot_length; pt3.y= (spot_wide + spot_distance) - spot_wide/2; pt3.z= 0.02;
	convex_hull3.points.push_back(pt3);
	pt3.x = - spot_length - spot_length; pt3.y= (spot_wide + spot_distance) + spot_wide/2; pt3.z= 0.02;
	convex_hull3.points.push_back(pt3);
	pfv3.set_convex_hull(convex_hull3);
	
	pfv4.set_convex_hull(convex_hull2);
	
	// ______________________
	//|                      |
	//|      PointCloud      |
	//|______________________|	
	//Point Cloud publications
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/pc_ahead", 1);
	cloud_pub2 = n.advertise<sensor_msgs::PointCloud2>("/pc_spot", 1);
	cloud_pub3 = n.advertise<sensor_msgs::PointCloud2>("/pc_behind", 1);
	cloud_pub4 = n.advertise<sensor_msgs::PointCloud2>("/pc_ground", 1);
	cloud_pub5 = n.advertise<sensor_msgs::PointCloud2>("/pc_parking_location", 1);

	// ______________________
	//|                      |
	//|      PCL subscr.     |
	//|______________________|
	// Creates a ROS subscriber for the input point cloud
	ros::Subscriber sub = n.subscribe ("/pc_out_pointcloud", 1, cloud_cb);
	
	ros::Rate loop_rate(30);
	ros::spin();
}
