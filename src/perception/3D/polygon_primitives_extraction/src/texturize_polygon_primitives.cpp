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
 * @addtogroup texturize_polygon_primitives 
 * @{
 * @file 
 * @brief Main code for adding texture to geometric polygonal primitives
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _texturize_polygon_primitives_CPP_
#define _texturize_polygon_primitives_CPP_


#include "texturize_polygon_primitives.h"

#define _USE_THREADS_ 0

void handler_cam_roof_fc(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cam_roof_fc.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cam_roof_fc.image_ts = msg->header.stamp;
	cam_roof_fc.msg_received = true;
	cam_roof_fc.image = cam_roof_fc.cv_ptr->image;//copy the cam images
}

void handler_cam_roof_fl(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cam_roof_fl.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cam_roof_fl.image_ts = msg->header.stamp;
	cam_roof_fl.msg_received = true;
	cam_roof_fl.image = cam_roof_fl.cv_ptr->image;//copy the cam images
}

void handler_cam_roof_fr(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cam_roof_fr.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cam_roof_fr.image_ts = msg->header.stamp;
	cam_roof_fr.msg_received = true;
	cam_roof_fr.image = cam_roof_fr.cv_ptr->image;//copy the cam images
}

void handler_cam_roof_rc(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cam_roof_rc.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cam_roof_rc.image_ts = msg->header.stamp;
	cam_roof_rc.msg_received = true;
	cam_roof_rc.image = cam_roof_rc.cv_ptr->image;//copy the cam images
}

void handler_cam_roof_fc_6mm(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cam_roof_fc_6mm.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cam_roof_fc_6mm.image_ts = msg->header.stamp;
	cam_roof_fc_6mm.msg_received = true;
	cam_roof_fc_6mm.image = cam_roof_fc_6mm.cv_ptr->image;//copy the cam images
}

int add_camera_projection_to_polygon(std::string cam_name, t_cam* cam, c_polygon_primitive_with_texture* plg)
{
	if (cam->msg_received)
	{
		ros::Time t = ros::Time::now();
		ROS_INFO("Building local mesh for camera %s to plg %s",cam_name.c_str(), plg->data.misc.name);

		//First step is to query for the camera transformation at timestamp 
		try
		{
			p_listener->lookupTransform("/tf_" + cam_name, "/world", cam->image_ts, cam->tf);
		}
		catch (tf::TransformException ex){ROS_ERROR("%s",ex.what()); return -1;}

		char str[1024];
		ros::Duration dur = ros::Time::now() - mission_start;
		sprintf(str,"{%s, %s, %0.1f}", plg->data.misc.name,cam_name.c_str(), dur.toSec());
		std::string projection_name = str; 
		//Now we map the camera projection to the polygon
		int v = plg->add_camera_projection_known_camera(&cam->image, cam->image_ts, cam->tf, cam_name, projection_name, plg->colormap->cv_color(plg->cp.size()));

		if (v!=-1)
		{
			ros::Duration d = ros::Time::now() - t;
			ROS_INFO("Successfully build of local mesh (%ld faces) for camera projection %s to plg %s (%f seconds)",plg->cp[v].mesh.number_of_faces(), cam_name.c_str(),plg->data.misc.name, d.toSec());
			return v;
		}
		else
		{
			ROS_INFO("Could not build local mesh for camera %s to plg %s",cam_name.c_str(),plg->data.misc.name);
			return v;
		}
	}
	else
	{
		ROS_INFO("Did not receive any new image from camera %s. Cannot create local mesh",cam_name.c_str());
		return -1;
	}

	return -1;
}


void* process_polygon_primitive(void* ptr)
{
	c_polygon_primitive_with_texture* plg = (c_polygon_primitive_with_texture*) ptr;

#if _USE_THREADS_
	ROS_INFO("\n__________________________________\nStart processing thread plg %s\n__________________________________\n", plg->data.misc.name);
	pthread_mutex_lock(&plg->mutex);
#endif
	//-------------------------------------------
	//Compute the camera's projections to the polygon plg
	//-------------------------------------------
	add_camera_projection_to_polygon("cam_roof_fc_6mm", &cam_roof_fc_6mm, plg);
	//add_camera_projection_to_polygon("cam_roof_fc", &cam_roof_fc, plg);
	//add_camera_projection_to_polygon("cam_roof_fr", &cam_roof_fr, plg);
	//add_camera_projection_to_polygon("cam_roof_fl", &cam_roof_fl, plg);
	//add_camera_projection_to_polygon("cam_roof_rc", &cam_roof_rc, plg);

	//-------------------------------------------
	//Build the global mesh for this particular polygon
	//-------------------------------------------
	plg->build_global_mesh(p_markerarray_pub);
#if _USE_THREADS_
	pthread_mutex_unlock(&plg->mutex);
	ROS_INFO("\n__________________________________\nFinished processing thread plg %s\n__________________________________\n", plg->data.misc.name);
#endif
	return NULL;
}

/**
 * @brief Handles the receiving of a new polygonal primitive message
 *
 * @param input a reference for the polygonal primitive data structure
 */
void polygon_primitive_received_handler(const polygon_primitive_msg::polygon_primitive& input)
{

	ros::Duration(0.1).sleep();

	ROS_INFO("Received a new polygon primitive msg. Polygon name %s", input.name.c_str());

	//----------------------------------------------------
	//Skip some polygon if required 
	//----------------------------------------------------
	if (input.name!="p1")
		return;

	//----------------------------------------------------
	//Find the key of this polygon in the map structure. The pointer plg will point to the polygon
	//----------------------------------------------------
	c_polygon_primitive_with_texture* plg = NULL;
	std::map<std::string, c_polygon_primitive_with_texture>::iterator it;
	it = pmap.find(input.name);

	//----------------------------------------------------
	//a key with this id was found. Must adapt the polygon
	//----------------------------------------------------
	if (it != pmap.end()) 
	{
		ROS_INFO("have a key with code %s . Will addapt polygon", input.name.c_str());

		it->second.readapt_to_new_plane((polygon_primitive_msg::polygon_primitive*)&input); //adapt the polygon to new plane

		//for (size_t i=0; i<it->second.cp.size(); i++)
		//{
		//printf("RE adding projection camera %d to mesh\n",(int)i);
		//it->second.dp.add_point_cloud(&it->second.cp[i].vertex_projectable, &it->second.cp[i].vertex_projectable_weight, i);
		//}

		plg = &it->second; //Set the pointer to the polygon in the map were using

	}
	else //no key found, should add to map
	{
		ROS_INFO("have no key with code %s . Will add polygon to pmap", input.name.c_str());

		//Create a polygon from the polygon_msg
		c_polygon_primitive_with_texture new_plg_from_msg(p_node);
		new_plg_from_msg.import_from_polygon_primitive_msg((polygon_primitive_msg::polygon_primitive*) &input);

		//Add the polygon to the map
		std::pair< std::map<std::string, c_polygon_primitive_with_texture>::iterator, bool> ret;
		ret = pmap.insert(std::pair<std::string, c_polygon_primitive_with_texture>(input.name, new_plg_from_msg));
		if (ret.second==false)
		{
			plg=NULL;
			ROS_ERROR("Error, could not insert polygon to map");
		}
		else
		{
			plg = &ret.first->second;	
		}
	}

#if _USE_THREADS_
	pthread_create(&plg->thread,NULL,process_polygon_primitive,(void*) plg);
#else
	process_polygon_primitive((void*)plg);
#endif
}

/**
 * @brief The main function for the extraction of polygon primitives
 * @param argc the standard argc
 * @param argv the standard argv
 * @return standard return for main 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "texturize_polygon_primitives"); // Initialize ROS coms
	ros::NodeHandle n; //The node handle
	p_node = &n;
	ros::Rate r(10);
	ros::Rate r1(1);

	cv::startWindowThread();

	tf::TransformListener listener;
	p_listener = &listener;

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub_fc = it.subscribe("/cam_roof_fc", 1, handler_cam_roof_fc);
	while(n.ok()){ros::spinOnce(); if (cam_roof_fc.msg_received) break; ROS_INFO("Waiting for first topic /cam_roof_fc");r1.sleep();}

	image_transport::Subscriber sub_fl = it.subscribe("/cam_roof_fl", 1, handler_cam_roof_fl);
	//while(n.ok()){ros::spinOnce(); if (cam_roof_fl.msg_received) break; ROS_INFO("Waiting for first topic /cam_roof_fl");r1.sleep();}

	image_transport::Subscriber sub_fr = it.subscribe("/cam_roof_fr", 1, handler_cam_roof_fr);
	//while(n.ok()){ros::spinOnce(); if (cam_roof_fr.msg_received) break; ROS_INFO("Waiting for first topic /cam_roof_fr");r1.sleep();}

	image_transport::Subscriber sub_rc = it.subscribe("/cam_roof_rc", 1, handler_cam_roof_rc);
	//while(n.ok()){ros::spinOnce(); if (cam_roof_rc.msg_received) break; ROS_INFO("Waiting for first topic /cam_roof_rc");r1.sleep();}

	image_transport::Subscriber sub_fc_6mm = it.subscribe("/cam_roof_fc_6mm", 1, handler_cam_roof_fc_6mm);
	//while(n.ok()){ros::spinOnce(); if (cam_roof_fc_6mm.msg_received) break; ROS_INFO("Waiting for first topic /cam_roof_fc_6mm");r1.sleep();}

	ros::Subscriber sub_polygon_primitive = n.subscribe("/polygon_primitive", 10, polygon_primitive_received_handler);  
	ros::Publisher markerarray_pub = n.advertise<visualization_msgs::MarkerArray>("/PolygonMarkers_with_texture", 5);
	p_markerarray_pub = &markerarray_pub;

	ROS_INFO("Waiting for topic /polygon_primitive");
	mission_start = ros::Time::now();

	ros::Time t=ros::Time::now();
	while(n.ok())
	{
		ros::Duration d = ros::Time::now()-t;
		//ROS_INFO("duration = %f", d.toSec());
		if (d.toSec()>5)
		{
			//-------------------------------------------
			//Send new info to rviz
			//-------------------------------------------
			std::vector<visualization_msgs::Marker> marker_vec;
			visualization_msgs::MarkerArray marker_array_msg;
			std::map<std::string, c_polygon_primitive_with_texture>::iterator it;

			ROS_INFO("Sending textured polygon primitives to RVIZ");
			for (it=pmap.begin(); it!=pmap.end();++it)
			{
				visualization_msgs::MarkerArray marker_vec;
			//	visualization_msgs::MarkerArray marker_array_msg;


#if _USE_THREADS_
				if (pthread_mutex_trylock(&it->second.mutex)) //check mutex
				{
#endif
					ROS_INFO("Sending tpp %s", it->second.data.misc.name);
					it->second.create_textures_vizualization_msg(&marker_vec, false);
					//marker_array_msg.set_markers_vec(marker_vec);	
					p_markerarray_pub->publish(marker_vec);

					for (size_t i=0; i<it->second.cp.size();i++)
					{
						cv::namedWindow(it->second.cp[i].projection_name);
						cv::imshow(it->second.cp[i].projection_name,it->second.cp[i].image_gui);	
						//cv::namedWindow(it->second.cp[i].projection_name+"canny");
						//cv::imshow(it->second.cp[i].projection_name+"canny",it->second.cp[i].mask_projectable_pixels);	
					}

#if _USE_THREADS_
					pthread_mutex_unlock(&it->second.mutex);
				}
				else
				{
					ROS_WARN("Could not send tpp %s. Thread locked", it->second.data.misc.name);
				}
#endif

			}

			cv::waitKey(50);				

			t=ros::Time::now();
		}

		r.sleep();
		ros::spinOnce();
	}

	PFLN

	return 0;
}
#endif
/**
 *@}
 */

