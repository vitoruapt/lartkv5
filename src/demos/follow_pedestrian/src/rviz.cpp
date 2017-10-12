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
 * @file rviz.cpp
 * @brief Planar scan generator
 * @author Miguel Oliveira
 * @version v0
 * @date 2012-02-29
 */

#ifndef _RVIZ_CPP_
#define _RVIZ_CPP_

#include "follow_pedestrian.h"

void add_to_viz_markers_vec(std::vector<visualization_msgs::Marker>* marker_vec)
{
	visualization_msgs::Marker marker;
	geometry_msgs::Point p;
	static ros::Time trigger_blink=ros::Time::now();
	static bool flip=false;

	//Trigger zone cylinder
	marker.header.frame_id = "/atc/vehicle/ground";
	marker.header.stamp = ros::Time();
	marker.ns = "trigger_zone";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = search_area.x;
	marker.pose.position.y = search_area.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;

	if ((ros::Time::now()-trigger_blink).toSec() > 1)
	{
		flip=!flip;
		trigger_blink = ros::Time::now();
	}

	if (flip)
	{
		marker.color.r = 0.5;
		marker.color.g = 0.5;
		marker.color.b = 0.0;
	}
	else
	{
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 0.0;
	}
	marker_vec->push_back(marker);

	//Trigger zone Text
	marker.header.frame_id = "/atc/vehicle/ground";
	marker.header.stamp = ros::Time();
	marker.ns = "trigger_text";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = search_area.x;
	marker.pose.position.y = search_area.y;
	marker.pose.position.z = 0.3;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = search_area.radius;
	marker.scale.y = search_area.radius;
	marker.scale.z = 0.3;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	if (status.state==SEARCHING)
		marker.text = "SEARCHING: Waiting for someone";
	else if (status.state==TRACKING)
	{
		char str[1024];
		sprintf(str,"TRACKING: Following target id %d",status.target_id);
		marker.text = str;
	}
	else if (status.state==TRACKING_NOT_SAFE)
	{
		char str[1024];
		sprintf(str,"TRACKING_NOT_SAFE: Waiting for clearance to follow target id %d",status.target_id);
		marker.text = str;
	}

	marker_vec->push_back(marker);

	//Safety zone lines
	marker.header.frame_id = "/atc/vehicle/ground";
	marker.header.stamp = ros::Time();
	marker.ns = "safety_zone";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 1;
	marker.scale.z = 0.2;
	marker.color.a = 1.0;

	if (!is_safe_using_lasers())//replace here by the danger flag
	{
		marker.color.r = 1;
		marker.color.g = 0;
		marker.color.b = 0;
	}
	else
	{
		marker.color.r = 0;
		marker.color.g = 1;
		marker.color.b = 0;
	}

	p.x = safety_zone.xmin;	p.y = safety_zone.ymin;	p.z = 0;
	marker.points.push_back(p);

	p.x = safety_zone.xmin;	p.y = safety_zone.ymax;	p.z = 0;
	marker.points.push_back(p);
	marker.points.push_back(p);

	p.x = safety_zone.xmax;	p.y = safety_zone.ymax;	p.z = 0;
	marker.points.push_back(p);
	marker.points.push_back(p);

	p.x = safety_zone.xmax;	p.y = safety_zone.ymin;	p.z = 0;
	marker.points.push_back(p);
	marker.points.push_back(p);

	p.x = safety_zone.xmin;	p.y = safety_zone.ymin;	p.z = 0;
	marker.points.push_back(p);

	marker_vec->push_back(marker);

	//send the obj model for the pedestrian marker
	marker.id = 1;	
	marker.header.stamp = ros::Time::now();	
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;	
	marker.action = visualization_msgs::Marker::ADD;	
	marker.ns = "pedestrian";
	marker.mesh_use_embedded_materials = 1;	
	marker.header.frame_id = "/atc/vehicle/ground";
	marker.scale.x = .4;
	marker.scale.y = .4;
	marker.scale.z = .4;
	marker.color.r = 0.2;
	marker.color.g = 0.3;
	marker.color.b = 0.4;
	marker.color.a = 1;
	marker.mesh_resource = "package://wrapper_collada/models/decisive_woman.obj";
	marker.header.stamp = ros::Time::now();	

	if (status.state==TRACKING || status.state == TRACKING_NOT_SAFE)
	{
		marker.pose.position.x = status.current_x;
		marker.pose.position.y = status.current_y;
		marker.pose.position.z = status.current_z;
		marker.pose.orientation = status.current_q;
	}
	else
	{
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 3000;
	}

	marker_vec->push_back(marker);
}


#endif
