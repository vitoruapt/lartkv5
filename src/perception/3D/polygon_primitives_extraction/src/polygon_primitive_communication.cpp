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
 * @addtogroup polygon_primitive 
 * @{
 * @file 
 * @brief All ros communication functions are defined here.
 * @note IS deprecated
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_communication_CPP_
#define _polygon_primitive_communication_CPP_


#include "polygon_primitive.h"


int c_polygon_primitive::export_to_polygon_primitive_msg(polygon_primitive_msg::polygon_primitive* msg)
{
	//Header
	msg->header.stamp = ros::Time::now();
	msg->header.frame_id = data.frames.global_name;

	//polygon name
	msg->name = std::string(data.misc.name);

	//Frame names
	msg->frame_global_name = data.frames.global_name;
	msg->frame_local_name = data.frames.local_name;

	//Frames
	msg->frame_current.translation.x = (data.frames.current.transform.getOrigin()).x();
	msg->frame_current.translation.y = (data.frames.current.transform.getOrigin()).y();
	msg->frame_current.translation.z = (data.frames.current.transform.getOrigin()).z();
	
	msg->frame_current.rotation.x = (data.frames.current.transform.getRotation()).x();
	msg->frame_current.rotation.y = (data.frames.current.transform.getRotation()).y();
	msg->frame_current.rotation.z = (data.frames.current.transform.getRotation()).z();
	msg->frame_current.rotation.w = (data.frames.current.transform.getRotation()).w();
	
	msg->frame_previous.translation.x = (data.frames.previous.transform.getOrigin()).x();
	msg->frame_previous.translation.y = (data.frames.previous.transform.getOrigin()).y();
	msg->frame_previous.translation.z = (data.frames.previous.transform.getOrigin()).z();
	
	msg->frame_previous.rotation.x = (data.frames.previous.transform.getRotation()).x();
	msg->frame_previous.rotation.y = (data.frames.previous.transform.getRotation()).y();
	msg->frame_previous.rotation.z = (data.frames.previous.transform.getRotation()).z();
	msg->frame_previous.rotation.w = (data.frames.previous.transform.getRotation()).w();

	//polygon color
	msg->color_r = data.misc.color.r;
	msg->color_g = data.misc.color.g;
	msg->color_b = data.misc.color.b;

	//convex hull
	pcl::toROSMsg(*data.hulls.convex.polygon, msg->convex_hull);
	pcl::toROSMsg(*data.hulls.convex.extended_polygon, msg->extended_convex_hull);
	msg->convex_hull_area = data.hulls.convex.area;
	msg->convex_hull_solidity = data.hulls.convex.solidity;

	//concave hull
	pcl::toROSMsg(*data.hulls.concave.polygon, msg->concave_hull);
	pcl::toROSMsg(*data.hulls.concave.extended_polygon, msg->extended_concave_hull);
	msg->concave_hull_area = data.hulls.concave.area;
	msg->concave_hull_solidity = data.hulls.concave.solidity;

	//plane
	msg->support_plane_A = data.planes.current->values[0];
	msg->support_plane_B = data.planes.current->values[1];
	msg->support_plane_C = data.planes.current->values[2];
	msg->support_plane_D = data.planes.current->values[3];

	return 1;
}

int c_polygon_primitive::import_from_polygon_primitive_msg(polygon_primitive_msg::polygon_primitive* msg)
{

	//polygon name
	sprintf(data.misc.name, "%s",msg->name.c_str());

	//Frame names
	data.frames.global_name = msg->frame_global_name;
	data.frames.local_name = msg->frame_local_name;

	//Frames
	data.frames.current.transform = tf::Transform(tf::Quaternion(msg->frame_current.rotation.x, msg->frame_current.rotation.y, msg->frame_current.rotation.z, msg->frame_current.rotation.w),  
			tf::Vector3(msg->frame_current.translation.x, msg->frame_current.translation.y, msg->frame_current.translation.z));

	data.frames.previous.transform = tf::Transform(tf::Quaternion(msg->frame_previous.rotation.x, msg->frame_previous.rotation.y, msg->frame_previous.rotation.z, msg->frame_previous.rotation.w),  
			tf::Vector3(msg->frame_previous.translation.x, msg->frame_previous.translation.y, msg->frame_previous.translation.z));

	//polygon color
	data.misc.color.r = msg->color_r;
	data.misc.color.g = msg->color_g;
	data.misc.color.b = msg->color_b;

	//convex hull
	pcl::fromROSMsg(msg->convex_hull, *data.hulls.convex.polygon);
	pcl::fromROSMsg(msg->extended_convex_hull, *data.hulls.convex.extended_polygon);
	data.hulls.convex.area = msg->convex_hull_area;
	data.hulls.convex.solidity = msg->convex_hull_solidity;

	//concave hull
	pcl::fromROSMsg(msg->concave_hull, *data.hulls.concave.polygon);
	pcl::fromROSMsg(msg->extended_concave_hull, *data.hulls.concave.extended_polygon);
	data.hulls.concave.area = msg->concave_hull_area;
	data.hulls.concave.solidity = msg->concave_hull_solidity;

	//plane
	data.planes.current->values[0] = msg->support_plane_A;
	data.planes.current->values[1] = msg->support_plane_B;
	data.planes.current->values[2] = msg->support_plane_C;
	data.planes.current->values[3] = msg->support_plane_D;

	return 1;
}
/**
 * @brief Publishes the local coordinate system with respect to the /world
 */
void c_polygon_primitive::publish_local_tf(void)
{
	char str[1024];
	sprintf(str, "/%s",data.misc.name);

	br.sendTransform(tf::StampedTransform(data.frames.current.transform, ros::Time::now(), data.frames.global_name, data.frames.local_name));   
}

visualization_msgs::Marker c_polygon_primitive::create_visualization_marker_header(
		std::string frame_id, ros::Time stamp, std::string name,
		int action, int id, int type,
		double px, double py, double pz,
		double qx, double qy, double qz, double qw,
		double sx, double sy, double sz,
		double cr, double cg, double cb, double alpha
		)
{
	visualization_msgs::Marker mk;
	mk.header.frame_id = frame_id; mk.header.stamp = stamp;	mk.ns = name;
	mk.action = action;	mk.type = type; mk.id = id;
	mk.pose.position.x = px; mk.pose.position.y = py; mk.pose.position.z = pz;
	mk.pose.orientation.x = qx;	mk.pose.orientation.y = qy;	mk.pose.orientation.z = qz;	mk.pose.orientation.w = qw;
	mk.scale.x = sx; mk.scale.y = sy; mk.scale.z = sz; 
	mk.color.r = cr; mk.color.g = cg; mk.color.b = cb; mk.color.a = alpha;
	return mk;
};


int c_polygon_primitive::create_vizualization_msgs(visualization_msgs::MarkerArray* marker_vec, unsigned int id_start)
{
	
	unsigned int id=id_start;

	geometry_msgs::Point p;
	std_msgs::ColorRGBA color;

	if (true) //Polygon name marker (TEXT_VIEW_FACING)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"name",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::TEXT_VIEW_FACING,
				data.frames.current.origin.x, data.frames.current.origin.y, data.frames.current.origin.z,
				0,0,0,1,
				1,1,0.6,
				data.misc.color.r/255., data.misc.color.g/255., data.misc.color.b/255., 1
				);

		char str[1024];
		sprintf(str,"%s A=%3.2f S=%3.2f",data.misc.name, data.hulls.convex.area, data.hulls.convex.solidity);
		std::string tmp_str(str); 
		msg.text = tmp_str; 
		marker_vec->markers.push_back(msg);
	}

	if (true) //concave hull marker (LINE_STRIP)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"concave_hull",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::LINE_STRIP,
				0,0,0,
				0,0,0,1,
				0.05,1,1,
				//data.misc.color.r/255., data.misc.color.g/255., data.misc.color.b/255., 1
				1,0,0, 1
				);

		if ((int)data.hulls.concave.polygon->points.size() >0)
		{
			for (unsigned int u=0; u< data.hulls.concave.polygon->size(); u++)
			{
				p.x = data.hulls.concave.polygon->points[u].x; 
				p.y = data.hulls.concave.polygon->points[u].y; 
				p.z = data.hulls.concave.polygon->points[u].z; 
				msg.points.push_back(p);
			}
			p.x = data.hulls.concave.polygon->points[0].x; 
			p.y = data.hulls.concave.polygon->points[0].y; 
			p.z = data.hulls.concave.polygon->points[0].z; 
			msg.points.push_back(p);
		}
		else
		{
			p.x = 0; p.y = 0; p.z = 0;
			msg.points.push_back(p);
			p.x = 1; p.y = 1; p.z = 1;
			msg.points.push_back(p);
		}

		marker_vec->markers.push_back(msg);
	}


	if ((int)data.hulls.convex.polygon->points.size() >1) //convex hull marker (LINE_STRIP)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"convex_hull",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::LINE_STRIP,
				0,0,0,
				0,0,0,1,
				0.15,1,1,
				data.misc.color.r/255., data.misc.color.g/255., data.misc.color.b/255., 1
				);

		for (unsigned int u=0; u< data.hulls.convex.polygon->size(); u++)
		{
			p.x = data.hulls.convex.polygon->points[u].x; 
			p.y = data.hulls.convex.polygon->points[u].y; 
			p.z = data.hulls.convex.polygon->points[u].z; 
			msg.points.push_back(p);
		}
		p.x = data.hulls.convex.polygon->points[0].x; 
		p.y = data.hulls.convex.polygon->points[0].y; 
		p.z = data.hulls.convex.polygon->points[0].z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	if ((int)data.hulls.convex.extended_polygon->points.size() >1) //extended convex hull marker (LINE_STRIP)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"extended_convex_hull",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::LINE_STRIP,
				0,0,0,
				0,0,0,1,
				0.05,1,1,
				data.misc.color.r/255./4, data.misc.color.g/255./4, data.misc.color.b/255./4, 1
				);

		for (unsigned int u=0; u< data.hulls.convex.extended_polygon->size(); u++)
		{
			p.x = data.hulls.convex.extended_polygon->points[u].x; 
			p.y = data.hulls.convex.extended_polygon->points[u].y; 
			p.z = data.hulls.convex.extended_polygon->points[u].z; 
			msg.points.push_back(p);
		}
		p.x = data.hulls.convex.extended_polygon->points[0].x; 
		p.y = data.hulls.convex.extended_polygon->points[0].y; 
		p.z = data.hulls.convex.extended_polygon->points[0].z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	if (true) //reference system x axis (ARROW)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"current_frame",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::ARROW,
				0,0,0,
				0,0,0,1,
				0.2,0.5,1,
				1, 0, 0, 1
				);

		p.x = data.frames.current.origin.x; 
		p.y = data.frames.current.origin.y; 
		p.z = data.frames.current.origin.z; 
		msg.points.push_back(p);

		p.x = data.frames.current.arrow_x.x; 
		p.y = data.frames.current.arrow_x.y; 
		p.z = data.frames.current.arrow_x.z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	if (true) //reference system y axis (ARROW)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"current_frame",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::ARROW,
				0,0,0,
				0,0,0,1,
				0.2,0.5,1,
				0, 1, 0, 1
				);

		p.x = data.frames.current.origin.x; 
		p.y = data.frames.current.origin.y; 
		p.z = data.frames.current.origin.z; 
		msg.points.push_back(p);

		p.x = data.frames.current.arrow_y.x; 
		p.y = data.frames.current.arrow_y.y; 
		p.z = data.frames.current.arrow_y.z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	if (true) //reference system z axis (ARROW)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"current_frame",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::ARROW,
				0,0,0,
				0,0,0,1,
				0.2,0.5,1,
				0, 0, 1, 1
				);

		p.x = data.frames.current.origin.x; 
		p.y = data.frames.current.origin.y; 
		p.z = data.frames.current.origin.z; 
		msg.points.push_back(p);

		p.x = data.frames.current.arrow_z.x; 
		p.y = data.frames.current.arrow_z.y; 
		p.z = data.frames.current.arrow_z.z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	// ------------------------------------------
	//add a marker to the old frame
	// ------------------------------------------

	if (true) //grow system x axis (ARROW)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"previous_frame",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::ARROW,
				0,0,0,
				0,0,0,1,
				0.2,0.5,1,
				0.3, 0, 0, 0.6
				);

		p.x = data.frames.previous.origin.x; 
		p.y = data.frames.previous.origin.y; 
		p.z = data.frames.previous.origin.z; 
		msg.points.push_back(p);

		p.x = data.frames.previous.arrow_x.x; 
		p.y = data.frames.previous.arrow_x.y; 
		p.z = data.frames.previous.arrow_x.z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	if (true) //grow system y axis (ARROW)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"previous_frame",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::ARROW,
				0,0,0,
				0,0,0,1,
				0.2,0.5,1,
				0, 0.3, 0, 0.6
				);

		p.x = data.frames.previous.origin.x; 
		p.y = data.frames.previous.origin.y; 
		p.z = data.frames.previous.origin.z; 
		msg.points.push_back(p);

		p.x = data.frames.previous.arrow_y.x; 
		p.y = data.frames.previous.arrow_y.y; 
		p.z = data.frames.previous.arrow_y.z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	if (true) //grow system z axis (ARROW)
	{
		visualization_msgs::Marker msg = create_visualization_marker_header(
				data.frames.global_name, ros::Time::now(),"previous_frame",
				visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::ARROW,
				0,0,0,
				0,0,0,1,
				0.2,0.5,1,
				0, 0, 0.3, 0.6
				);

		p.x = data.frames.previous.origin.x; 
		p.y = data.frames.previous.origin.y; 
		p.z = data.frames.previous.origin.z; 
		msg.points.push_back(p);

		p.x = data.frames.previous.arrow_z.x; 
		p.y = data.frames.previous.arrow_z.y; 
		p.z = data.frames.previous.arrow_z.z; 
		msg.points.push_back(p);

		marker_vec->markers.push_back(msg);
	}

	return 1;
}

#endif
/**
 *@}
 */      
