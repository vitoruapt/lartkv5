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
 * @addtogroup polygon_primitive_with_texture 
 * @{
 * @file 
 * @brief Holds the c_polygon_primitive rviz drawing methods
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_with_texture_rviz_CPP_
#define _polygon_primitive_with_texture_rviz_CPP_


#include "polygon_primitive_with_texture.h"

int c_polygon_primitive_with_texture::erase_old_markers(visualization_msgs::MarkerArray* marker_vec, unsigned int from, unsigned int to, std::string namesp)
{
	for (unsigned int j=from; j<to; j++) //erase old markers
	{
		visualization_msgs::Marker msg; //declare the msg 
		msg.header.frame_id = data.frames.global_name;
		msg.header.stamp = ros::Time::now();
		msg.ns = namesp.c_str();
		msg.id = j;
		msg.action = visualization_msgs::Marker::DELETE;
		marker_vec->markers.push_back(msg);
	}	
	return 1;
}

int c_polygon_primitive_with_texture::create_textures_vizualization_msg(visualization_msgs::MarkerArray* marker_vec, bool reset_id)
{

	//compute the id_start for this polygon based on its name
	std::string str;
	str = data.misc.name;
	str.erase(str.begin());
	id_start = atoi(str.c_str())*100; 

	unsigned int id_range;


	geometry_msgs::Point p;
	std_msgs::ColorRGBA color;

	if(2)
	{	
		//Draw the camera positions as a ref system
		id_range = id_camera_position;
		id_camera_position=id_start;
		for (size_t j=0; j<cp.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_camera_position;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::LINE_STRIP;
			msg.id = id_camera_position++;

			msg.scale.x = 0.05; 
			msg.scale.y = 0.05; 
			msg.scale.z = 0.05; 
			msg.color = colormap->color(j);
			msg.color.a = 1;

			geometry_msgs::Point p_origin;
			p_origin.x = (cp[j].camera_6dof_position.inverse().getOrigin()).x(); 
			p_origin.y = (cp[j].camera_6dof_position.inverse().getOrigin()).y(); 
			p_origin.z = (cp[j].camera_6dof_position.inverse().getOrigin()).z(); 

			pcl::PointCloud<pcl::PointXYZ> pts_local;
			pcl::PointCloud<pcl::PointXYZ> pts;
			pcl::PointXYZ pt;

			pt.x=0.5; pt.y=0; pt.z=0; pts_local.push_back(pt);
			pt.x=0; pt.y=0.5; pt.z=0; pts_local.push_back(pt);
			pt.x=0; pt.y=0; pt.z=0.5; pts_local.push_back(pt);

			pcl_ros::transformPointCloud(pts_local, pts, cp[j].camera_6dof_position.inverse()); 

			msg.points.push_back(p_origin);
			p.x = pts.points[0].x; 
			p.y = pts.points[0].y; 
			p.z = pts.points[0].z; 
			msg.points.push_back(p);

			msg.points.push_back(p_origin);
			p.x = pts.points[1].x; 
			p.y = pts.points[1].y; 
			p.z = pts.points[1].z; 
			msg.points.push_back(p);

			msg.points.push_back(p_origin);
			p.x = pts.points[2].x; 
			p.y = pts.points[2].y; 
			p.z = pts.points[2].z; 
			msg.points.push_back(p);


			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_camera_position, id_range,ns_camera_position);
	}

	if(2)
	{	
		//Draw the projections names
		id_range = id_projection_name;
		id_projection_name=id_start;
		for (size_t j=0; j<cp.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_projection_name;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.id = id_projection_name++;

			msg.scale.x = 1; 
			msg.scale.y = 1; 
			msg.scale.z = 0.6; 
			msg.color = colormap->color(j);
			msg.color.a = 1;

			msg.pose.position.x = (cp[j].camera_6dof_position.inverse().getOrigin()).x(); 
			msg.pose.position.y = (cp[j].camera_6dof_position.inverse().getOrigin()).y(); 
			msg.pose.position.z = (cp[j].camera_6dof_position.inverse().getOrigin()).z(); 
			msg.text = cp[j].projection_name; 


			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_projection_name, id_range,ns_projection_name);
	}

	if(2)
	{	
		//Draw the camera projections
		id_range = id_camera_canvas;
		id_camera_canvas=id_start;
		for (size_t j=0; j<cp.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_camera_canvas;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::LINE_STRIP;
			msg.id = id_camera_canvas++;

			msg.scale.x = 0.1; 
			msg.color = colormap->color(j);
			msg.color.a = 1;

			for (size_t u=0; u< cp[j].vertex_canvas.points.size(); u++)
			{
				p.x = cp[j].vertex_canvas.points[u].x; 
				p.y = cp[j].vertex_canvas.points[u].y; 
				p.z = cp[j].vertex_canvas.points[u].z; 

				if (!isnan(p.x)) msg.points.push_back(p);
			}

			p.x = cp[j].vertex_canvas.points[0].x; 
			p.y = cp[j].vertex_canvas.points[0].y; 
			p.z = cp[j].vertex_canvas.points[0].z; 

			if (!isnan(p.x)) msg.points.push_back(p);
			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_camera_canvas, id_range,ns_camera_canvas);
	}

	if(2)
	{	
		//Draw the camera projections polygon intersection
		id_range = id_camera_intersection;
		id_camera_intersection=id_start;
		for (size_t j=0; j<cp.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_camera_intersection;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::LINE_STRIP;
			msg.id = id_camera_intersection++;

			msg.scale.x = 0.1; 
			msg.color = colormap->color(j);
			msg.color.a =1;

			for (size_t u=0; u< cp[j].vertex_intersection.points.size(); u++)
			{
				p.x = cp[j].vertex_intersection.points[u].x; 
				p.y = cp[j].vertex_intersection.points[u].y; 
				p.z = cp[j].vertex_intersection.points[u].z; 

				if (!isnan(p.x)) msg.points.push_back(p);

			}
			p.x = cp[j].vertex_intersection.points[0].x; 
			p.y = cp[j].vertex_intersection.points[0].y; 
			p.z = cp[j].vertex_intersection.points[0].z; 

			if (!isnan(p.x)) msg.points.push_back(p);
			marker_vec->markers.push_back(msg);

		}
		erase_old_markers(marker_vec, id_camera_intersection, id_range,ns_camera_intersection);
	}

	if(2)
	{	
		//Draw the camera projections polygon intersection vertices
		id_range = id_camera_intersection_vertices;
		id_camera_intersection_vertices=id_start;
		for (size_t j=0; j<cp.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_camera_intersection_vertices;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::POINTS;
			msg.id = id_camera_intersection_vertices++;

			msg.scale.x = 0.15; 
			msg.scale.y = 0.15; 
			msg.color = colormap->color(j);
			msg.color.a =1;

			for (size_t u=0; u< cp[j].vertex_intersection.points.size(); u++)
			{
				p.x = cp[j].vertex_intersection.points[u].x; 
				p.y = cp[j].vertex_intersection.points[u].y; 
				p.z = cp[j].vertex_intersection.points[u].z; 

				if (!isnan(p.x)) msg.points.push_back(p);
			}

			if (!isnan(p.x)) msg.points.push_back(p);
			marker_vec->markers.push_back(msg);

		}
		erase_old_markers(marker_vec, id_camera_intersection_vertices, id_range,ns_camera_intersection_vertices);
	}

	if(1)
	{	
		//Draw the Triangles with color
		id_range = id_textured_triangles;
		id_textured_triangles=id_start;
		if (dp.pc.points.size()>=3)
		{

			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_textured_triangles;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
			msg.id = id_textured_triangles++;

			msg.pose.position.x = 0;
			msg.pose.position.y = 0;
			msg.pose.position.z = 0;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;

			msg.scale.x = 1; 
			msg.scale.y = 1; 
			msg.scale.z = 1; 
			msg.color.r = data.misc.color.r/255.;
			msg.color.g = data.misc.color.g/255.;
			msg.color.b = data.misc.color.b/255.; 
			msg.color.a = 1;
			for (int u=0; u<(int)dp.pc.points.size(); u++)
			{
				p.x = dp.pc.points[u].x; 
				p.y = dp.pc.points[u].y; 
				p.z = dp.pc.points[u].z; 
				msg.points.push_back(p);

				uint32_t rgb = *reinterpret_cast<int*>(&dp.pc.points[u].rgb);
				uint8_t r = (rgb >> 16) & 0x0000ff;
				uint8_t g = (rgb >> 8)  & 0x0000ff;
				uint8_t b = (rgb)     & 0x0000ff;

				color.r = (float)r/255.0;
				color.g = (float)g/255.0;
				color.b = (float)b/255.0;
				color.a =1;
				msg.colors.push_back(color);

			}

			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_textured_triangles, id_range,ns_textured_triangles);
	}

	if(2)
	{	
		//Draw the Triangles edges
		id_range = id_triangle_edges;
		id_triangle_edges=id_start;
		//if (ts.pc.points.size()>=3)

		//printf("dp.pc.points.size()=%d\n",(int)dp.pc.points.size());
		//printf("dp.pcproveniences.size()=%d\n",(int)dp.pc_proveniences.size());
		if (dp.pc.points.size()>=3)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_triangle_edges;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::LINE_LIST;
			msg.id = id_triangle_edges++;

			msg.pose.position.x = 0;
			msg.pose.position.y = 0;
			msg.pose.position.z = 0;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;

			msg.scale.x = 0.002; 
			msg.scale.y = 1; 
			msg.scale.z = 1; 
			msg.color.r = 0/255.;
			msg.color.g = 255/255.;
			msg.color.b = 0/255.; 
			msg.color.a = 0.6;

			//for (int u=0; u<(int)ts.pc.points.size(); u+=3)
			for (int u=0; u<(int)dp.pc.points.size(); u+=3)
			{
				color = colormap->color(dp.pc_proveniences[u]);
				p.x = dp.pc.points[u].x; 
				p.y = dp.pc.points[u].y; 
				p.z = dp.pc.points[u].z; 
				msg.points.push_back(p);
				msg.colors.push_back(color);

				p.x = dp.pc.points[u+1].x; 
				p.y = dp.pc.points[u+1].y; 
				p.z = dp.pc.points[u+1].z; 
				msg.points.push_back(p);
				msg.colors.push_back(color);



				p.x = dp.pc.points[u+1].x; 
				p.y = dp.pc.points[u+1].y; 
				p.z = dp.pc.points[u+1].z; 
				msg.points.push_back(p);
				msg.colors.push_back(color);


				p.x = dp.pc.points[u+2].x; 
				p.y = dp.pc.points[u+2].y; 
				p.z = dp.pc.points[u+2].z; 
				msg.points.push_back(p);
				msg.colors.push_back(color);


				p.x = dp.pc.points[u].x; 
				p.y = dp.pc.points[u].y; 
				p.z = dp.pc.points[u].z; 
				msg.points.push_back(p);
				msg.colors.push_back(color);

				p.x = dp.pc.points[u+2].x; 
				p.y = dp.pc.points[u+2].y; 
				p.z = dp.pc.points[u+2].z; 
				msg.points.push_back(p);
				msg.colors.push_back(color);
			}

			marker_vec->markers.push_back(msg);

		}
		erase_old_markers(marker_vec, id_triangle_edges, id_range,ns_triangle_edges);
	}

	if(2)
	{	
		//Draw the Triangles proveniences with color
		id_range = id_proveniences;
		id_proveniences=id_start;
		//if (ts.pc.points.size()>=3)
		if (dp.pc.points.size()>=3)
		{
			visualization_msgs::Marker msg; //declare the msg                                              
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_proveniences;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;
			msg.scale.z = 0.01; 

			//printf("dp.pc_faces_visited.points.size()=%d\n",(int)dp.pc_faces_visited.points.size());
			//printf("dp.pc.points.size()=%d\n",(int)dp.pc.points.size());

			for (int u=0; u<(int)dp.pc.points.size(); u+=3)
			{
				msg.id = id_proveniences++;
				msg.pose.position.x = (dp.pc.points[u].x + dp.pc.points[u+1].x + dp.pc.points[u+2].x)/3; ;
				msg.pose.position.y = (dp.pc.points[u].y + dp.pc.points[u+1].y + dp.pc.points[u+2].y)/3; ;
				msg.pose.position.z = (dp.pc.points[u].z + dp.pc.points[u+1].z + dp.pc.points[u+2].z)/3; ;
				msg.color = colormap->color(dp.pc_proveniences[u]);
				msg.color.a = 1;

				if (dp.pc_proveniences[u]<0)
				{
					msg.text="NA";	
				}
				else
				{
					char str[1024];
					sprintf(str, "%d", dp.pc_proveniences[u]);
					msg.text=str;	
				}

				marker_vec->markers.push_back(msg);
			}
		}
		erase_old_markers(marker_vec, id_proveniences, id_range,ns_proveniences);
	}

	if(2)
	{	
		//Draw the Triangles vertices 
		//id_range = id_triangle_vertices;
		//id_triangle_vertices=id_start;
		//if (ts.pc.points.size()>=3)
		//if (dp.pc.points.size()>=3)
		//{
		//visualization_msgs::Marker msg; //declare the msg 
		//msg.header.frame_id = data.frames.global_name;
		//msg.header.stamp = ros::Time::now();
		//msg.ns = ns_triangle_vertices;
		//msg.action = visualization_msgs::Marker::ADD;
		//msg.type = visualization_msgs::Marker::POINTS;
		//msg.id = id_triangle_vertices++;

		//msg.pose.position.x = 0;
		//msg.pose.position.y = 0;
		//msg.pose.position.z = 0;
		//msg.pose.orientation.x = 0.0;
		//msg.pose.orientation.y = 0.0;
		//msg.pose.orientation.z = 0.0;
		//msg.pose.orientation.w = 1.0;

		//msg.scale.x = 0.05; 
		//msg.scale.y = 0.05; 
		//msg.scale.z = 1; 
		//msg.color.r = 1.;
		//msg.color.g = 0;
		//msg.color.b = 1;
		//msg.color.a = 1;

		////for (int u=0; u<(int)ts.pc.points.size(); u++)
		////{
		////p.x = ts.pc.points[u].x; 
		////p.y = ts.pc.points[u].y; 
		////p.z = ts.pc.points[u].z; 
		////msg.points.push_back(p);

		////std_msgs::ColorRGBA color1 = colormap->color(ts.pc_proveniences[u]);
		////color1.a = 0.5;
		////msg.colors.push_back(color1);
		////}

		//for (int u=0; u<(int)dp.pc.points.size(); u++)
		//{
		//p.x = dp.pc.points[u].x; 
		//p.y = dp.pc.points[u].y; 
		//p.z = dp.pc.points[u].z; 
		//msg.points.push_back(p);

		//std_msgs::ColorRGBA color1 = colormap->color(dp.pc_proveniences[u]);
		//color1.a = 0.5;
		//msg.colors.push_back(color1);
		//}

		//marker_vec->markers.push_back(msg);

		//}
		//erase_old_markers(marker_vec, id_triangle_vertices, id_range,ns_triangle_vertices);
	}

	if(2)
	{	
		//Draw the projection union
		id_range = id_projection_union;
		id_projection_union=id_start;
		for (size_t j=0; j<dp.projection_union.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_projection_union;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::LINE_STRIP;
			msg.id = id_projection_union++;

			msg.scale.x = 0.2; 
			msg.color.r = 1.;
			msg.color.g = 0;
			msg.color.b = 1;
			msg.color.a = 0.7;

			for (size_t u=0; u< dp.projection_union[j].points.size(); u++)
			{
				p.x = dp.projection_union[j].points[u].x; 
				p.y = dp.projection_union[j].points[u].y; 
				p.z = dp.projection_union[j].points[u].z; 

				if (!isnan(p.x)) msg.points.push_back(p);

			}

			p.x = dp.projection_union[j].points[0].x; 
			p.y = dp.projection_union[j].points[0].y; 
			p.z = dp.projection_union[j].points[0].z; 

			if (!isnan(p.x)) msg.points.push_back(p);
			marker_vec->markers.push_back(msg);

		}
		erase_old_markers(marker_vec, id_projection_union, id_range,ns_projection_union);
	}

	if(2)
	{	
		//Draw the projection union vertices
		id_range = id_projection_union_vertices;
		id_projection_union_vertices=id_start;
		for (size_t j=0; j<dp.projection_union.size(); j++)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_projection_union_vertices;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::POINTS;
			msg.id = id_projection_union_vertices++;

			msg.scale.x = 0.3; 
			msg.scale.y = 0.3; 
			msg.scale.z = 1; 
			msg.color.r = 1.;
			msg.color.g = 0;
			msg.color.b = 1;
			msg.color.a = 0.7;

			for (size_t u=0; u< dp.projection_union[j].points.size(); u++)
			{
				p.x = dp.projection_union[j].points[u].x; 
				p.y = dp.projection_union[j].points[u].y; 
				p.z = dp.projection_union[j].points[u].z; 

				if (!isnan(p.x)) msg.points.push_back(p);

			}

			if (!isnan(p.x)) msg.points.push_back(p);
			marker_vec->markers.push_back(msg);

		}
		erase_old_markers(marker_vec, id_projection_union_vertices, id_range,ns_projection_union_vertices);

		//Draw the constraints
		id_range = id_constraints;
		id_constraints=id_start;
		//ROS_WARN("size of pc_constraints=%d", (int)dp.pc_constraints.points.size());
		if(dp.pc_constraints.points.size()>1) 
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_constraints;
			msg.action = visualization_msgs::Marker::ADD;
			msg.pose.orientation.w = 1.0;
			msg.type = visualization_msgs::Marker::LINE_LIST;
			msg.id = id_constraints++;

			msg.scale.x = 0.01; 
			msg.color.r = 1.;
			msg.color.g = 0;
			msg.color.b = 0;
			msg.color.a = 0.8;

			for (size_t u=0; u< dp.pc_constraints.points.size(); u+=2)
			{
				p.x = (dp.pc_constraints.points[u+1].x - dp.pc_constraints.points[u].x); 
				p.y = (dp.pc_constraints.points[u+1].y - dp.pc_constraints.points[u].y); 
				p.z = (dp.pc_constraints.points[u+1].z - dp.pc_constraints.points[u].z); 


				geometry_msgs::Point p1,p2;

				p1.x = dp.pc_constraints.points[u].x + 0.4*p.x;	p1.y = dp.pc_constraints.points[u].y + 0.4*p.y;	p1.z = dp.pc_constraints.points[u].z + 0.4*p.z;
				p2.x = dp.pc_constraints.points[u].x + 0.6*p.x;	p2.y = dp.pc_constraints.points[u].y + 0.6*p.y;	p2.z = dp.pc_constraints.points[u].z + 0.6*p.z;

				if (!isnan(p.x)) msg.points.push_back(p1);
				if (!isnan(p.x)) msg.points.push_back(p2);
			}
			marker_vec->markers.push_back(msg);

		}
		erase_old_markers(marker_vec, id_constraints, id_range,ns_constraints);
	}

	if(2)
	{	
		//Draw the local meshes
		//id_range = id_local_mesh;
		//id_local_mesh=id_start;
		//for (size_t j=0; j<cp.size(); j++)
		//{
		//visualization_msgs::Marker msg; //declare the msg 
		//msg.header.frame_id = data.frames.global_name;
		//msg.header.stamp = ros::Time::now();
		//msg.ns = ns_local_mesh;
		//msg.action = visualization_msgs::Marker::ADD;
		//msg.type = visualization_msgs::Marker::LINE_LIST;
		//msg.id = id_local_mesh++;

		//msg.pose.orientation.w = 1.0;

		//msg.scale.x = 0.01; 
		//msg.scale.y = 1; 
		//msg.scale.z = 1; 
		//msg.color = colormap->color(j);
		//msg.color.a = 0.5;

		//for (int u=0; u<(int)cp[j].vertex_list.points.size(); u+=3)
		//{
		//if (isnan(cp[j].vertex_list.points[u].x) || isnan(cp[j].vertex_list.points[u+1].x) || isnan(cp[j].vertex_list.points[u+2].x) ||
		//isnan(cp[j].vertex_list.points[u].y) || isnan(cp[j].vertex_list.points[u+1].y) || isnan(cp[j].vertex_list.points[u+2].y) ||
		//isnan(cp[j].vertex_list.points[u].z) || isnan(cp[j].vertex_list.points[u+1].z) || isnan(cp[j].vertex_list.points[u+2].z))
		//continue;

		//p.x = cp[j].vertex_list.points[u].x; 
		//p.y = cp[j].vertex_list.points[u].y; 
		//p.z = cp[j].vertex_list.points[u].z; 
		//msg.points.push_back(p);

		//p.x = cp[j].vertex_list.points[u+1].x; 
		//p.y = cp[j].vertex_list.points[u+1].y; 
		//p.z = cp[j].vertex_list.points[u+1].z; 
		//msg.points.push_back(p);

		//p.x = cp[j].vertex_list.points[u].x; 
		//p.y = cp[j].vertex_list.points[u].y; 
		//p.z = cp[j].vertex_list.points[u].z; 
		//msg.points.push_back(p);

		//p.x = cp[j].vertex_list.points[u+2].x; 
		//p.y = cp[j].vertex_list.points[u+2].y; 
		//p.z = cp[j].vertex_list.points[u+2].z; 
		//msg.points.push_back(p);

		//p.x = cp[j].vertex_list.points[u+1].x; 
		//p.y = cp[j].vertex_list.points[u+1].y; 
		//p.z = cp[j].vertex_list.points[u+1].z; 
		//msg.points.push_back(p);

		//p.x = cp[j].vertex_list.points[u+2].x; 
		//p.y = cp[j].vertex_list.points[u+2].y; 
		//p.z = cp[j].vertex_list.points[u+2].z; 
		//msg.points.push_back(p);

		//}

		//ROS_INFO("Projection %d Number of points = %d", (int)j, (int)cp[j].vertex_list.points.size());
		//marker_vec->markers.push_back(msg);

		//}
		//erase_old_markers(marker_vec, id_local_mesh, id_range,ns_local_mesh);
	}

	if(2)
	{	
		//Draw the next triangle
		id_range = id_next_triangle;
		id_next_triangle=id_start;
		//if (ts.pc.points.size()>=3)
		if (next_triangle.points.size()==6)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_next_triangle;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::LINE_LIST;
			msg.id = id_next_triangle++;

			msg.pose.position.x = 0;
			msg.pose.position.y = 0;
			msg.pose.position.z = 0;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;

			msg.scale.x = 0.01; 
			msg.scale.y = 1; 
			msg.scale.z = 1; 
			msg.color.r = 0;
			msg.color.g = 1;
			msg.color.b = 0; 
			msg.color.a = 0.8;

			for (int u=0; u<(int)next_triangle.points.size(); u++)
			{
				p.x = next_triangle.points[u].x; 
				p.y = next_triangle.points[u].y; 
				p.z = next_triangle.points[u].z; 
				msg.points.push_back(p);
			}
			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_next_triangle, id_range,ns_next_triangle);
	}

	if(2)
	{	
		//Draw the edge 0
		id_range = id_edge0;
		id_edge0=id_start;
		if (next_triangle.points.size()==6)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_edge0;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.id = id_edge0++;

			msg.pose.position.x = next_triangle.points[0].x + 0.05;
			msg.pose.position.y = next_triangle.points[0].y;
			msg.pose.position.z = next_triangle.points[0].z;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;
			msg.scale.z = 0.05; 
			msg.color.r = 0; msg.color.g = 1; msg.color.b = 0; msg.color.a = 0.8;
			msg.text = "0";

			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_edge0, id_range,ns_edge0);
	}

	if(2)
	{	
		//Draw the edge 1
		id_range = id_edge1;
		id_edge1=id_start;
		if (next_triangle.points.size()==6)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_edge1;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.id = id_edge1++;

			msg.pose.position.x = next_triangle.points[1].x + 0.05;
			msg.pose.position.y = next_triangle.points[1].y;
			msg.pose.position.z = next_triangle.points[1].z;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;
			msg.scale.z = 0.05; 
			msg.color.r = 0; msg.color.g = 1; msg.color.b = 0; msg.color.a = 0.8;
			msg.text = "1";

			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_edge1, id_range,ns_edge1);
	}

	if(2)
	{	
		//Draw the edge 2
		id_range = id_edge2;
		id_edge2=id_start;
		if (next_triangle.points.size()==6)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_edge2;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.id = id_edge2++;

			msg.pose.position.x = next_triangle.points[5].x + 0.05;
			msg.pose.position.y = next_triangle.points[5].y;
			msg.pose.position.z = next_triangle.points[5].z;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;
			msg.scale.z = 0.05; 
			msg.color.r = 0; msg.color.g = 1; msg.color.b = 0; msg.color.a = 0.8;
			msg.text = "2";

			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_edge2, id_range,ns_edge2);
	}

	if(2)
	{	
		//vertices_indices 
		id_range = id_vertices_indices;
		id_vertices_indices=id_start;
		for (size_t i=0; i<dp.pc_vertices.size(); i++)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_vertices_indices;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.id = id_vertices_indices++;
			msg.scale.z = 0.01; 
			msg.color.r = 1; msg.color.g = 0; msg.color.b = 0; msg.color.a = 1;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;

			msg.pose.position.x = dp.pc_vertices.points[i].x;
			msg.pose.position.y = dp.pc_vertices.points[i].y;
			msg.pose.position.z = dp.pc_vertices.points[i].z;
			char str[1024];
			sprintf(str, "%d",(int)dp.pc_vertices_indices[i]);
			msg.text = str;
			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_vertices_indices, id_range,ns_vertices_indices);
	}

	if(2)
	{	
		//visited_faces 
		id_range = id_visited_faces;
		id_visited_faces=id_start;
		for (size_t i=0; i<dp.pc_faces_visited.points.size(); i++)
		{
			visualization_msgs::Marker msg; //declare the msg 
			msg.header.frame_id = data.frames.global_name;
			msg.header.stamp = ros::Time::now();
			msg.ns = ns_visited_faces;
			msg.action = visualization_msgs::Marker::ADD;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.id = id_visited_faces++;
			msg.scale.z = 0.01; 
			msg.color.r = 0/255.;
			msg.color.g = 128/255.;
			msg.color.b = 128/255.; 
			msg.color.a = 0.8;
			msg.pose.orientation.x = 0.0;
			msg.pose.orientation.y = 0.0;
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 1.0;

			msg.pose.position.x = dp.pc_faces_visited.points[i].x;
			msg.pose.position.y = dp.pc_faces_visited.points[i].y;
			msg.pose.position.z = dp.pc_faces_visited.points[i].z;
			msg.text = "__V";
			marker_vec->markers.push_back(msg);
		}
		erase_old_markers(marker_vec, id_visited_faces, id_range,ns_visited_faces);
	}
	return 1;
}






#endif
/**
 *@}
 */      
