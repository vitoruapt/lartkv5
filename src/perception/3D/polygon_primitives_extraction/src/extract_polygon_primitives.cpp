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
 * @addtogroup extract_polygon_primitives 
 *@{
 * @file 
 * @brief The main file of the extract polygon primitives binary
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _extract_polygon_primitives_CPP_
#define _extract_polygon_primitives_CPP_


#include <vector>
#include <fstream>
#include <stdio.h>

#include "extract_polygon_primitives.h"


//#define _VERTICAL_PLG_LONG_EXPANSION_ 0.4
//#define _VERTICAL_PLG_PERP_EXPANSION_ 0.6
#define _VERTICAL_PLG_LONG_EXPANSION_ 0.2
#define _VERTICAL_PLG_PERP_EXPANSION_ 0.5
#define _GROUND_PLG_LONG_EXPANSION_ 15 
#define _GROUND_PLG_PERP_EXPANSION_ 0.3

#define _MAX_ITERATION_TIME_ 20.0
#define DEBUG 0
#define USE_EXPANSION 1

bool polygon_has_quality(double area, double solidity)
{
	//SET 1
	if (area > 7)
	{
		if (solidity >48)
			return true;
		else
			return false;
	}
	else
		return false;


	//SET 2
	//if (area > 1 && solidity> 2)
	//{
	//return true;
	//}
	//else
	//{
	//return false;
	//}
}


int publish_point_cloud(std::vector<boost::shared_ptr<ros::Publisher> >* publishers, pcl::PointCloud<pcl::PointXYZ>* pc, std::string topic_name)
{
	int index=-1;
	for (size_t i=0; publishers->size(); ++i)
	{
		if ((*publishers)[i]->getTopic()==topic_name)	
		{
			index=i;
			break;
		}
	}

	if (index==-1)
		ROS_ERROR("Topic name %s not in the publishers list. Cannot publish.", topic_name.c_str());

	sensor_msgs::PointCloud2 cloud_msg;  
	pcl::toROSMsg(*pc, cloud_msg);
	(*publishers)[index]->publish(cloud_msg);
	return 1;
}

int publish_point_cloud(std::vector<boost::shared_ptr<ros::Publisher> >* publishers, pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::string topic_name)
{

	pcl::PointCloud<pcl::PointXYZ> cloud_tmp; //the normals of that pc
	cloud_tmp = *pc;
	return publish_point_cloud(publishers, &cloud_tmp, topic_name);
}
/**
 * @brief The main function for the extraction of polygon primitives
 * @param argc the standard argc
 * @param argv the standard argv
 * @return standard return for main 
 */
int main(int argc, char **argv)
{
	//-------- Set initial value for variables ---------------
	flag_msg_received = false;


	std::vector<std::string> label;
	label.push_back((const char*)"A");	
	label.push_back((const char*)"B");	
	label.push_back((const char*)"C");	
	label.push_back((const char*)"D");	
	label.push_back((const char*)"E");	
	label.push_back((const char*)"F");	
	label.push_back((const char*)"G");	
	label.push_back((const char*)"H");	
	label.push_back((const char*)"I");	

	std::vector<double> r_time_detection;
	std::vector<double> r_time_expansion;
	std::vector<double> r_time_total;
	std::vector<size_t> r_input_pts;
	std::vector<size_t> r_input_pts_after_expansion;
	std::vector<size_t> r_input_pts_after_detection;
	std::vector<size_t> r_input_pts_explained;
	std::vector<size_t> r_input_pts_ground_explained;
	std::vector<size_t> r_area;
	std::vector<size_t> r_ground_area;
	std::vector<size_t> r_num_plg;
	std::vector<size_t> r_num_searches;

	//--------------- Declare variables ----------------------
	sensor_msgs::PointCloud2 cloud_msg;  
	pcl::PointCloud<pcl::PointXYZ> cloud_to_process; //the tmp pc to be processed
	pcl::PointCloud<pcl::PointXYZ> cloud_to_process1; //the tmp pc to be processed
	pcl::PointCloud<pcl::PointXYZ> cloud_elim; //the tmp pc to be processed
	pcl::PointCloud<pcl::PointNormal> cloud_normals; 
	pcl::PointCloud<pcl::Normal> normals; //the normals of that pc
	std::vector<c_polygon_primitive> plg; //Create a vector of polygons

	//--------------- Initialize stuff  ----------------------
	ros::init(argc, argv, "extract_polygon_primitives"); // Initialize ROS coms
	ros::NodeHandle n; //The node handle
	pn = &n;
	ros::Rate r(0.1);
	ros::Subscriber sub_point_cloud = n.subscribe("/filtered_cloud", 1, PointCloudHandler);  


	boost::shared_ptr<ros::Publisher> pub1(new ros::Publisher); 
	*pub1 = n.advertise<sensor_msgs::PointCloud2>("/epp_pc_all", 1);
	publishers.push_back(pub1);

	boost::shared_ptr<ros::Publisher> pub2(new ros::Publisher); 
	*pub2 = n.advertise<sensor_msgs::PointCloud2>("/epp_pc_filtered", 1);
	publishers.push_back(pub2);

#if DEBUG
	for (int i=0; i<_MAX_NUM_POLYGONS_; ++i)
	{
		char str[1024];
		sprintf(str,"/inliers_%d",i);
		boost::shared_ptr<ros::Publisher> pub_tmp(new ros::Publisher); 
		*pub_tmp = n.advertise<sensor_msgs::PointCloud2>(str, 1);
		publishers.push_back(pub_tmp);

		sprintf(str,"/projected_%d",i);
		boost::shared_ptr<ros::Publisher> pub_tmp1(new ros::Publisher); 
		*pub_tmp1 = n.advertise<sensor_msgs::PointCloud2>(str, 1);
		publishers.push_back(pub_tmp1);

	}
#endif

	ros::Publisher normals_pub = n.advertise<visualization_msgs::Marker>("/normals", 1);
	ros::Publisher markerarray_pub = n.advertise<visualization_msgs::MarkerArray>("/PolygonMarkers", 1);
	visualization_msgs::MarkerArray marker_array_msg; //declare the array
	visualization_msgs::Marker marker;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/gpp_mesh", 1);

	ros::Publisher pub_polygon_primitive = n.advertise<polygon_primitive_msg::polygon_primitive>("/polygon_primitive", 1);
	tf::TransformListener listener;
	tf::StampedTransform vehicle_tf;
	for (int j=0; j<10; j++)
		ros::spinOnce();

	unsigned int count=0;
	int iteration=0;

	//--------------- main cycle ----------------------
	while(n.ok())
	{
		if (flag_msg_received) //if a pc msg is received lets process it
		{

			ros::Time t = ros::Time::now();
			flag_msg_received=0; //reset the flag variable
			cloud_to_process = cloud; // copy cloud to cloud_to_process

			ROS_INFO("New Velodyne pc received. %ld points", cloud_to_process.points.size());

			r_input_pts.push_back(cloud_to_process.points.size());
#if DEBUG
			//publish_point_cloud(&publishers, &cloud_to_process, "/epp_pc_all");
#endif


			ros::Time t_expansion = ros::Time::now();
#if USE_EXPANSION
			//   ___________________________________
			//   |                                 |
			//   |       Polygon Expansion         |
			//   |_________________________________| 
			for (int i=0; i<(int)plg.size();i++)
			{
				if (i==0)
					plg[i].polygon_expansion(&cloud_to_process, _GROUND_PLG_LONG_EXPANSION_, _GROUND_PLG_PERP_EXPANSION_);
				else
					plg[i].polygon_expansion(&cloud_to_process, _VERTICAL_PLG_LONG_EXPANSION_, _VERTICAL_PLG_PERP_EXPANSION_);

				polygon_primitive_msg::polygon_primitive polygon_primitive_msg;
				plg[i].export_to_polygon_primitive_msg(&polygon_primitive_msg);
				pub_polygon_primitive.publish(polygon_primitive_msg);

#if 1
				//publish_point_cloud(&publishers, &cloud_to_process, "/epp_pc_all");

				create_publish_marker_array(&plg, &marker_array_msg );
				markerarray_pub.publish(marker_array_msg);
#endif
			}	
#endif
			ros::Duration d_expansion = ros::Time::now()-t_expansion;

			//Listen to transforms to obtain current vehicle position
			get_vehicle_position(&n, &listener, &vehicle_tf, &laser_ts);

			// Compute the normals for the cloud_to_process
			compute_normals(&cloud_to_process, vehicle_tf.getOrigin().x(),vehicle_tf.getOrigin().y(),vehicle_tf.getOrigin().z(),0,30, &normals);
#if DEBUG
			//save_pc_PointNormal_to_pcd(&cloud_to_process, &normals, std::string("cloud.pcd"));


			//publish a marker to rviz with the normals
			//visualization_msgs::Marker mk; //declare the array
			//geometry_msgs::Point p;
			//std_msgs::ColorRGBA color;

			//mk.header.frame_id = "/world"; mk.header.stamp = ros::Time::now();	mk.ns = "normals";
			//mk.action = visualization_msgs::Marker::ADD;	mk.type = visualization_msgs::Marker::LINE_LIST; mk.id = 0;
			//mk.pose.position.x = 0; mk.pose.position.y = 0; mk.pose.position.z = 0;
			//mk.pose.orientation.x = 0;	mk.pose.orientation.y = 0;	mk.pose.orientation.z = 0;	mk.pose.orientation.w = 1;
			//mk.scale.x = 0.01; mk.scale.y = 1; mk.scale.z = 1; 
			//mk.color.r = 1; mk.color.g = 0; mk.color.b = 0; mk.color.a = 1;

			//for (size_t i=0; i< cloud_to_process.points.size(); i+=25)
			//{
			//p.x = cloud_to_process.points[i].x;
			//p.y = cloud_to_process.points[i].y;
			//p.z = cloud_to_process.points[i].z;
			//mk.points.push_back(p);	

			//double rat=0.5;
			//p.x = cloud_to_process.points[i].x + (normals.points[i].normal[0])*rat;
			//p.y = cloud_to_process.points[i].y + (normals.points[i].normal[1])*rat;
			//p.z = cloud_to_process.points[i].z + (normals.points[i].normal[2])*rat;
			//mk.points.push_back(p);	
			//}

			//normals_pub.publish(mk);

			//filter_uncoherent_points(cloud_to_process.makeShared(), &cloud_to_process1, &cloud_elim, normals.makeShared() ,0.4, vehicle_tf.getOrigin().x(),vehicle_tf.getOrigin().y(),vehicle_tf.getOrigin().z());
			//publish_point_cloud(&publishers, &cloud_elim, "/epp_pc_filtered");
#endif

			cloud_to_process1 = cloud_to_process;


			ros::Time t_detection = ros::Time::now();
			r_input_pts_after_expansion.push_back(cloud_to_process.points.size());
			int ret=0;

			int num_searches = 0;
			//   ___________________________________
			//   |                                 |
			//   |       Polygon Detection         |
			//   |_________________________________| 
			int times=-1;
			bool to_break=false;
			for (int i=(int)plg.size(); i<_MAX_NUM_POLYGONS_;i++)
			{

				num_searches++;
#if DEBUG
				publish_point_cloud(&publishers, &cloud_to_process1, "/epp_pc_all");
#endif

				char str[1024];
				sprintf(str,"p%d",i);
				plg.push_back(*(new c_polygon_primitive(&n,str,cr[i%16], g[i%16],b[i%16])));

				if (i==0)
				{
					ret = plg[i].polygon_create(&cloud_to_process1, &normals, 0.3,0.2,100,0);
					plg[i].data.misc.ground_search = true;
				}
#if USE_EXPANSION<1
				else if (times<0)
				{
					ret = plg[i].polygon_create(&cloud_to_process1, &normals, 0.3,0.2,1000,0);
					plg[i].data.misc.ground_search = true;
				}
#endif
				else
				{
					ret = plg[i].polygon_create(&cloud_to_process1, &normals, 0.2,0.7,1000,1);
					plg[i].data.misc.ground_search = false;
					//ret = plg[i].polygon_create(&cloud_to_process1, &normals, 0.2,0.3,350,1);
				}
				times++;

				if ((ros::Time::now()-t).toSec()>_MAX_ITERATION_TIME_)
				{
					ROS_INFO("Stoping detection: %f seconds limit reached", _MAX_ITERATION_TIME_);
					to_break=true;
				}



				//ROS_INFO("After create: cloud_to_process %ld pts, normals %ld pts", cloud_to_process1.points.size(), normals.points.size());
				if (ret==0)
				{
					//delete &plg[i];
					//
					ROS_WARN("Deleting candidate polygon %d due to error in create",i);
					plg.erase(--plg.end());          
					i--;
					if(to_break)
						break;
					else
						continue;
				}


#if DEBUG
				//publish_point_cloud(&publishers, &cloud_to_process1, "/epp_pc_all");
#endif

				//Grow the polygon
				if (i==0)
					plg[i].polygon_expansion(&cloud_to_process1,  _GROUND_PLG_LONG_EXPANSION_,_GROUND_PLG_PERP_EXPANSION_ , &normals);
				else
					plg[i].polygon_expansion(&cloud_to_process1,  _VERTICAL_PLG_LONG_EXPANSION_, _VERTICAL_PLG_PERP_EXPANSION_, &normals);




				//ROS_INFO("After expansion: cloud_to_process %ld pts, normals %ld pts", cloud_to_process1.points.size(), normals.points.size());
				//create_publish_marker_array(&plg, &marker_array_msg );
				//markerarray_pub.publish(marker_array_msg);

#if EXPANSION <1
				if(times>0)
#else
				if (i!=0)
#endif
				{
					if (!polygon_has_quality(plg[i].data.hulls.convex.area, plg[i].data.hulls.convex.solidity))
					{
						ROS_WARN("Deleting candidate plg %d due to low area=%f solidity=%f",i,plg[i].data.hulls.convex.area, plg[i].data.hulls.convex.solidity);

						plg.erase(--plg.end());          

						i--;
						if(to_break)
							break;
						else
							continue;
					}
				}


				polygon_primitive_msg::polygon_primitive polygon_primitive_msg;
				plg[i].export_to_polygon_primitive_msg(&polygon_primitive_msg);
				pub_polygon_primitive.publish(polygon_primitive_msg);

#if 1
				create_publish_marker_array(&plg, &marker_array_msg );
				markerarray_pub.publish(marker_array_msg);
#endif


				ROS_INFO("Adding new plg %d with area=%f solidity=%f",i,plg[i].data.hulls.convex.area, plg[i].data.hulls.convex.solidity);
				if(to_break)
					break;
			}


			// data for matlab
			size_t n=0;
			size_t n1=0;
			size_t a=0;
			size_t a1=0;
			for (size_t i=0; i<plg.size(); ++i)
			{
				if (!plg[i].data.misc.ground_search)
				{
					n+= plg[i].pointclouds.all->points.size();
					a+= plg[i].data.hulls.convex.area;
				}
				else
				{
					n1+= plg[i].pointclouds.all->points.size();
					a1+= plg[i].data.hulls.convex.area;
				}
			}
			r_input_pts_explained.push_back(n);
			r_input_pts_ground_explained.push_back(n1);
			r_area.push_back(a);
			r_ground_area.push_back(a1);
			r_num_plg.push_back(plg.size());
			r_num_searches.push_back(num_searches);

			r_input_pts_after_detection.push_back(cloud_to_process1.points.size());
			ros::Duration d_detection = ros::Time::now()-t_detection;
			ros::Duration d = ros::Time::now()-t;


			create_publish_marker_array(&plg, &marker_array_msg );
			markerarray_pub.publish(marker_array_msg);

#if DEBUG
			publish_point_cloud(&publishers, &cloud_to_process1, "/epp_pc_all");

			for (size_t i=0; i<plg.size(); ++i)
			{
				char str[1024];
				sprintf(str,"/inliers_%ld",i);

				plg[i].pointclouds.all->height=1;
				publish_point_cloud(&publishers, plg[i].pointclouds.all, str);

				sprintf(str,"/projected_%ld",i);
				publish_point_cloud(&publishers, plg[i].pointclouds.projected, str);


			}
#endif

			for (size_t i=0; i<plg.size();i++)
			{
				plg[i].print_polygon_information();
			}

			//Publish the collada mesh_resource marker
			count++;
			char str[1024];
			sprintf(str,"mesh/polygons%d.dae",count);
			wrapper_collada wc(str);



			for (size_t i=1; i<plg.size();i++)
			{

				pcl::PointCloud<pcl::PointXYZ> normal;
				pcl::PointXYZ p;
				p.x = plg[i].data.planes.current->values[0];
				p.y = plg[i].data.planes.current->values[1];
				p.z = plg[i].data.planes.current->values[2];
				normal.points.push_back(p);
				std::string pname(plg[i].data.misc.name);
				wc.add_polygon_fixed_color_on_both_sides(pname,
						plg[i].data.hulls.convex.polygon,
						normal.makeShared(),
						(float)plg[i].data.misc.color.r/255.0,
						(float)plg[i].data.misc.color.g/255.0,
						(float)plg[i].data.misc.color.b/255.0,
						1.0);  

			}

			wc.write_file(); 	

			marker.id = 1;  
			marker.header.stamp = ros::Time::now(); 
			marker.type = visualization_msgs::Marker::MESH_RESOURCE;    
			marker.action = visualization_msgs::Marker::ADD;    
			marker.ns = "ns";
			char str1[1024];
			sprintf(str1,"package://polygon_primitives_extraction/bin/%s", str);
			marker.mesh_resource = str1;
			marker.mesh_use_embedded_materials = 1; 
			marker.header.frame_id = "/world";
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;
			marker.color.r = 0.2;
			marker.color.g = 0.2;
			marker.color.b = 0.2;
			marker.color.a = 1;	
			marker_pub.publish(marker); 
			//ros::Duration(2).sleep();	

			//Write OBJ mesh file format

			FILE* pFile;

			std::string ss = "./" + label[iteration] + ".obj";
			pFile = fopen (ss.c_str(),"w+");
			for (size_t i=0; i<plg.size();i++)
			{
				for (size_t j=0 ; j<plg[i].data.hulls.convex.polygon->points.size() ; ++j)
				{
					fprintf(pFile, "v %f %f %f\n",plg[i].data.hulls.convex.polygon->points[j].x,plg[i].data.hulls.convex.polygon->points[j].y,plg[i].data.hulls.convex.polygon->points[j].z);
				}
			}

			fprintf(pFile, "\n");
			int count =1;
			for (size_t i=0; i<plg.size();i++)
			{

				fprintf(pFile, "f ");
				for (size_t j=0 ; j<plg[i].data.hulls.convex.polygon->points.size() ; ++j)
				{
					fprintf(pFile, "%d ",count);
					count ++;
				}
				fprintf(pFile, "\n");
			}
			fclose (pFile);

			ss = "./" + label[iteration] + "_noground.obj";
			pFile = fopen (ss.c_str(),"w+");
			for (size_t i=1; i<plg.size();i++)
			{
				for (size_t j=0 ; j<plg[i].data.hulls.convex.polygon->points.size() ; ++j)
				{
					fprintf(pFile, "v %f %f %f\n",plg[i].data.hulls.convex.polygon->points[j].x,plg[i].data.hulls.convex.polygon->points[j].y,plg[i].data.hulls.convex.polygon->points[j].z);
				}
			}

			fprintf(pFile, "\n");
			count =1;
			for (size_t i=1; i<plg.size();i++)
			{

				fprintf(pFile, "f ");
				for (size_t j=0 ; j<plg[i].data.hulls.convex.polygon->points.size() ; ++j)
				{
					fprintf(pFile, "%d ",count);
					count ++;
				}
				fprintf(pFile, "\n");
			}
			fclose (pFile);

			iteration++;

			//pFile = fopen ("./tmp_concave.obj","w+");
			//for (size_t i=0; i<plg.size();i++)
			//{
			//for (size_t j=0 ; j<plg[i].data.hulls.concave.polygon->points.size() ; ++j)
			//{
			//fprintf(pFile, "v %f %f %f\n",plg[i].data.hulls.concave.polygon->points[j].x,plg[i].data.hulls.concave.polygon->points[j].y,plg[i].data.hulls.concave.polygon->points[j].z);
			//}
			//}

			//fprintf(pFile, "\n");
			//count =1;
			//for (size_t i=0; i<plg.size();i++)
			//{

			//fprintf(pFile, "f ");
			//for (size_t j=0 ; j<plg[i].data.hulls.concave.polygon->points.size() ; ++j)
			//{
			//fprintf(pFile, "%d ",count);
			//count ++;
			//}
			//fprintf(pFile, "\n");
			//}


			//ROS_INFO("Obj file written");
			//fclose (pFile);

			//pFile = fopen ("./tmp_concave_no_ground.obj","w+");
			//for (size_t i=1; i<plg.size();i++)
			//{
			//for (size_t j=0 ; j<plg[i].data.hulls.concave.polygon->points.size() ; ++j)
			//{
			//fprintf(pFile, "v %f %f %f\n",plg[i].data.hulls.concave.polygon->points[j].x,plg[i].data.hulls.concave.polygon->points[j].y,plg[i].data.hulls.concave.polygon->points[j].z);
			//}
			//}

			//fprintf(pFile, "\n");
			//count =1;
			//for (size_t i=1; i<plg.size();i++)
			//{

			//fprintf(pFile, "f ");
			//for (size_t j=0 ; j<plg[i].data.hulls.concave.polygon->points.size() ; ++j)
			//{
			//fprintf(pFile, "%d ",count);
			//count ++;
			//}
			//fprintf(pFile, "\n");
			//}


			//ROS_INFO("Obj file written");
			//fclose (pFile);

			//for (size_t i=0; i<plg.size();i++)
			//{
			//ROS_INFO("plg %ld Concave hull %ld points",i,plg[i].data.hulls.concave.polygon->points.size());
			//}


			ROS_INFO("Processing took %f secs",d.toSec());
			ROS_INFO("Expansion took %f secs",d_expansion.toSec());
			ROS_INFO("Detection took %f secs",d_detection.toSec());

			r_time_total.push_back(d.toSec());
			r_time_detection.push_back( d_detection.toSec());
			r_time_expansion.push_back(d_expansion.toSec());
		}


		//cv::waitKey(20);
		ros::spinOnce(); //do the ros spin
		ros::spinOnce(); //do the ros spin
		ros::spinOnce(); //do the ros spin
		ros::spinOnce(); //do the ros spin
	}

	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%s ",label[i].c_str());	
	}

	printf("\nTotal processing time\nA=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%f ",r_time_total[i]);	
	}

	printf("];\nDetection processing time\nB=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%f ",r_time_detection[i]);	
	}

	printf("];\nExpansion processing time\nC=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%f ",r_time_expansion[i]);	
	}

	printf("];\nInput pts\nD=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_input_pts[i]);	
	}

	printf("];\nInput pts after expansion\nE=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_input_pts_after_expansion[i]);	
	}

	printf("];\nInput pts after detection\nF=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_input_pts_after_detection[i]);	
	}

	printf("];\nExplained pts\nG=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_input_pts_explained[i]);	
	}

	printf("];\nGround Explained pts\nH=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_input_pts_ground_explained[i]);	
	}

	printf("];\nArea\nI=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_area[i]);	
	}

	printf("];\nGround Area\nJ=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_ground_area[i]);	
	}

	printf("];\nNumber of plg\nL=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_num_plg[i]);	
	}

	printf("];\nNumber of searches\nM=[");	
	for (size_t i=0; i<r_time_total.size(); i++)
	{
		printf("%ld ",r_num_searches[i]);	
	}

	printf("];\n");	


	//cv::destroyWindow("cam_roof_fc");
	return 0;
}

#endif
/**
 *@}
 */

