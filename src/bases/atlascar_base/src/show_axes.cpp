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
#ifndef _atlascar_transforms_CPP_
#define _atlascar_transforms_CPP_

/**
 * @file 
 * @brief A example code just to show all the onboar reference systems in the
 * atlascar
 * @author Miguel Oliveira
 */


#include <boost/algorithm/string.hpp>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/package.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <libxml/xmlreader.h>


#define PFLN {printf("FILE %s LINE %d\n",__FILE__, __LINE__);}

using namespace ros;
using namespace std;
using namespace boost;
using namespace visualization_msgs;

tf::TransformListener *p_listener;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
float marker_pos = 0;

typedef struct
{
	double x,y,z;
	std::string name;
}t_reference_frame;

std::vector<boost::shared_ptr <t_reference_frame> > v_rf;

int get_i_from_name(std::vector<boost::shared_ptr <t_reference_frame> >* v_rf, std::string name)
{
	int ret=-1; 
	for (size_t i=0; i<v_rf->size(); ++i)
	{
		if (name == (*v_rf)[i]->name)
		{
			ret=i;
			break;
		}	
	}

	return ret;
}

Marker makeBox( InteractiveMarker &msg )
{
	Marker marker;

	marker.type = Marker::SPHERE;
	marker.scale.x = msg.scale * 0.05;
	marker.scale.y = msg.scale * 0.05;
	marker.scale.z = msg.scale * 0.05;
	marker.color.r = 0;
	marker.color.g = 0.7;
	marker.color.b = 0;
	marker.color.a = 0.0;

	return marker;
}
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	switch ( feedback->event_type )
	{
		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			printf("you are moving marker %s\n",feedback->marker_name.c_str());


			int ret = get_i_from_name(&v_rf, feedback->marker_name);

			printf("ret=%d\n",ret);

			v_rf[ret]->x = feedback->pose.position.x;
			v_rf[ret]->y = feedback->pose.position.y;
			v_rf[ret]->z = feedback->pose.position.z;
			break;
	}

	server->applyChanges();
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
	InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back( makeBox(msg) );
	msg.controls.push_back( control );
	return msg.controls.back();
}

void set_label_pos(double x, double y, double z, visualization_msgs::Marker* msg, std::vector<visualization_msgs::Marker>* marker_vec)
{
	//tf::StampedTransform transform;
	//try
	//{
		//p_listener->lookupTransform("/atc/vehicle/ground", msg->header.frame_id, ros::Time(0), transform);
	//}
	//catch (tf::TransformException ex)
	//{
		//ROS_ERROR("%s",ex.what());
	//}


	//pcl::PointCloud<pcl::PointXYZ> global;
	//pcl::PointXYZ p;
	//p.x = x; p.y=y; p.z=z;
	//global.push_back(p);

	//pcl::PointCloud<pcl::PointXYZ> local;
	//pcl_ros::transformPointCloud(global,local,transform.inverse());

	geometry_msgs::Point pt;
	visualization_msgs::Marker lmsg; //declare the msg 
	lmsg.pose.orientation.w = 1.0;
	lmsg.header.stamp = ros::Time::now();
	lmsg.id = 0;
	lmsg.scale.x = 0.02; lmsg.scale.y = 1; lmsg.scale.z = 0.095; 
	lmsg.color.r = 0.0; lmsg.color.g = 0; lmsg.color.b = 0.0; lmsg.color.a = 0.4;
	lmsg.pose.position.x = 0; lmsg.pose.position.y = 0; lmsg.pose.position.z = 0; 
	lmsg.action = visualization_msgs::Marker::ADD;
	lmsg.type = visualization_msgs::Marker::LINE_STRIP;

	lmsg.header.frame_id = msg->header.frame_id;
	lmsg.ns = "CNT " + msg->header.frame_id;

	pt.x=0; pt.y=0; pt.z=0; 
	lmsg.points.push_back(pt);
	//pt.x=local.points[0].x; pt.y=local.points[0].y; pt.z=local.points[0].z;
	pt.x=x; pt.y=y; pt.z=z;
	lmsg.points.push_back(pt);
	marker_vec->push_back(lmsg);


	//msg->pose.position.x = local.points[0].x; msg->pose.position.y = local.points[0].y; msg->pose.position.z = local.points[0].z; 
	msg->pose.position.x = x; msg->pose.position.y = y; msg->pose.position.z = z; 
}




int add_to_viz_markers_vec(std::vector<visualization_msgs::Marker>* marker_vec)
{
	geometry_msgs::Point p;
	std_msgs::ColorRGBA color;

	//Draw the projections names
	visualization_msgs::Marker msg; //declare the msg 
	msg.pose.orientation.w = 1.0;
	msg.header.stamp = ros::Time::now();
	msg.id = 0;
	msg.scale.x = 1; msg.scale.y = 1; msg.scale.z = 0.1; 
	msg.color.r = 0.0; msg.color.g = 0; msg.color.b = 0.3; msg.color.a = 1;
	msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = -0.1; 
	msg.action = visualization_msgs::Marker::ADD;
	msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

	for (size_t i=0; i< v_rf.size(); ++i)
	{
		msg.header.frame_id = v_rf[i]->name;
		msg.ns = msg.header.frame_id.c_str();
		msg.text = msg.header.frame_id.c_str(); 
		set_label_pos(v_rf[i]->x,v_rf[i]->y,v_rf[i]->z, &msg, marker_vec);
		marker_vec->push_back(msg);
	}

	return 1;
}

void make6DofMarker( bool fixed, std::string name , double x, double y, double z)
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = name;
	int_marker.pose.position.x = x;
	int_marker.pose.position.y = y;
	int_marker.pose.position.z = z;
	int_marker.scale = 0.1;

	int_marker.name = name;
	int_marker.description = "";

	// insert a box
	makeBoxControl(int_marker);
	InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "move_z";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}

/**
@brief Get the name of a xml element

This function reads the name of the current xml element in the reader.
@param reader xmlTextReaderPtr pointing to the xml object
@param str variable that will store the name
*/
void GetName(xmlTextReaderPtr reader,string& str)
{
	xmlChar*name=xmlTextReaderName(reader);
	if(!name)return;
	str=(char*)name;
	xmlFree(name);
}

/**
@brief Get the value of a xml element

This function reads the value of the current xml element in the reader.
@param reader xmlTextReaderPtr pointing to the xml object
@param str variable that will store the value
*/
void GetValue(xmlTextReaderPtr reader,string& str)
{
	xmlChar*value=xmlTextReaderValue(reader);
	if(!value)return;
	str=(char*)value;
	xmlFree(value);
}

/**
@brief Get the value of an attribute in a xml element

This function reads the value of an attribute in the current xml element in the reader.
@param reader xmlTextReaderPtr pointing to the xml object
@param name name of the attribute to read
@param str variable that will store the value
*/
void GetAttribute(xmlTextReaderPtr reader,const xmlChar*name,string& str)
{
	xmlChar*attribute=xmlTextReaderGetAttribute(reader,name);
	if(!attribute)return;
	str=(char*)attribute;
	xmlFree(attribute);
}

void NodeHandler(xmlTextReaderPtr reader)
{
	string name,value;
	string dev_name;
	GetName(reader,name);
	
	to_upper(name);
	
	if(name=="LINK" && xmlTextReaderNodeType(reader)==1)
	{
		GetAttribute(reader,BAD_CAST "name",dev_name);
		boost::shared_ptr<t_reference_frame> rf(new t_reference_frame);
		rf->name = dev_name;
		rf->x = 0; rf->y =0; rf->z =0;
		v_rf.push_back(rf);
		make6DofMarker( true, v_rf[v_rf.size()-1]->name , v_rf[v_rf.size()-1]->x, v_rf[v_rf.size()-1]->y, v_rf[v_rf.size()-1]->z);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "atlascar_transforms"); // Initialize ROS coms
	ros::NodeHandle n; //The node handle
	ros::Rate loop_rate(1);	
	ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/axis_markers", 0 );
	tf::TransformListener listener(n,ros::Duration(10));
	p_listener=&listener;
	server.reset( new interactive_markers::InteractiveMarkerServer("show_axes","",false) );
	ros::Duration(0.1).sleep();



	//for now have a hard coded link to the atlascar urdf file, we could do this by the command line arguments
	//std::string urdf_file = ros::package::getPath("atlascar_base") + "/urdf/atlascar_urdf_3dsmodels.xml";

	std::string urdf_file = ros::package::getPath("coordinate_frames") + "/urdf/urdf.xml";

	printf("reading %s\n",urdf_file.c_str());

	int ret;
	xmlTextReaderPtr reader = xmlNewTextReaderFilename(urdf_file.c_str());
	if(reader!=NULL)
	{
		do
		{
			ret=xmlTextReaderRead(reader);//this function returns 1 if ok, -1 on error and 0 if there is no more nodes to read
			NodeHandler(reader);
		}while(ret==1);
		
		xmlFreeTextReader(reader);
		if (ret != 0)//zero means the end of the file
		{
			ROS_ERROR("Failed to parse file");
			return -1;
		}
	}else
	{
		ROS_ERROR("Failed to parse file");
		return -1;
	}

	PFLN





	server->applyChanges();


	while (ros::ok())
	{
		visualization_msgs::MarkerArray marker_msg;
		
		add_to_viz_markers_vec(&(marker_msg.markers));

		vis_pub.publish(marker_msg);

		// This will adjust as needed per iteration
		loop_rate.sleep();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
	}


	server.reset();
}
#endif
