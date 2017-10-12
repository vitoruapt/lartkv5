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
 * @{
 * @file 
 * @brief Implements functions to produce rviz markers
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _extract_polygon_primitives_RVIZMARKERS_CPP_
#define _extract_polygon_primitives_RVIZMARKERS_CPP_

/**
 */

#include "extract_polygon_primitives.h"

visualization_msgs::Marker create_visualization_marker_header(
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
}

void create_publish_marker_array(std::vector<c_polygon_primitive> *plg, visualization_msgs::MarkerArray *marker_array_msg)
{
	//Create and publish the marker array
	geometry_msgs::Point p;
	std_msgs::ColorRGBA color;
	std::vector<visualization_msgs::Marker> marker_vec;

	//for (int i=0, id=0; i<(int)plg->size();i++)
	for (int i=0, id=0; i<plg->size();i++)
	{


		(*plg)[i].create_vizualization_msgs(marker_array_msg, 0+100*i);
		continue;
		
		//if (true) //Polygon name marker (TEXT_VIEW_FACING)
		//{
			//visualization_msgs::Marker msg = create_visualization_marker_header(
					//(*plg)[i].data.frames.global_name, ros::Time::now(),"name",
					//visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::TEXT_VIEW_FACING,
					//(*plg)[i].data.frames.current.origin.x, (*plg)[i].data.frames.current.origin.y, (*plg)[i].data.frames.current.origin.z,
					//0,0,0,1,
					//1,1,0.6,
					//(*plg)[i].data.misc.color.r/255., (*plg)[i].data.misc.color.g/255., (*plg)[i].data.misc.color.b/255., 1
					//);

			//char str[1024];
			//sprintf(str,"%s A=%3.2f S=%3.2f",(*plg)[i].data.misc.name, (*plg)[i].data.hulls.convex.area, (*plg)[i].data.hulls.convex.solidity);
			//std::string tmp_str(str); 
			//msg.text = tmp_str; 
			//marker_vec.push_back(msg);
		//}


		//if (true) //concave hull marker (LINE_STRIP)
		//{
			//visualization_msgs::Marker msg = create_visualization_marker_header(
					//(*plg)[i].data.frames.global_name, ros::Time::now(),"concave_hull",
					//visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::LINE_STRIP,
					//0,0,0,
					//0,0,0,1,
					//0.05,1,1,
					////(*plg)[i].data.misc.color.r/255., (*plg)[i].data.misc.color.g/255., (*plg)[i].data.misc.color.b/255., 1
					//0, 1, 0,1 
					//);

			//if ((int)(*plg)[i].data.hulls.concave.polygon->points.size() >0)
			//{
				//for (unsigned int u=0; u< (*plg)[i].data.hulls.concave.polygon->size(); u++)
				//{
					//p.x = (*plg)[i].data.hulls.concave.polygon->points[u].x; 
					//p.y = (*plg)[i].data.hulls.concave.polygon->points[u].y; 
					//p.z = (*plg)[i].data.hulls.concave.polygon->points[u].z; 
					//msg.points.push_back(p);
				//}
				//p.x = (*plg)[i].data.hulls.concave.polygon->points[0].x; 
				//p.y = (*plg)[i].data.hulls.concave.polygon->points[0].y; 
				//p.z = (*plg)[i].data.hulls.concave.polygon->points[0].z; 
				//msg.points.push_back(p);
			//}
			//else
			//{
				//p.x = 0; p.y = 0; p.z = 0;
				//msg.points.push_back(p);
				//p.x = 1; p.y = 1; p.z = 1;
				//msg.points.push_back(p);
			//}

			//ROS_INFO("msg r=%f g=%f b=%f", msg.color.r, msg.color.g, msg.color.b);
			//marker_vec.push_back(msg);
		//}


		//if ((int)(*plg)[i].data.hulls.convex.polygon->points.size() >1) //convex hull marker (LINE_STRIP)
		//{
			//visualization_msgs::Marker msg = create_visualization_marker_header(
					//(*plg)[i].data.frames.global_name, ros::Time::now(),"convex_hull",
					//visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::LINE_STRIP,
					//0,0,0,
					//0,0,0,1,
					//0.05,1,1,
					////(*plg)[i].data.misc.color.r/255., (*plg)[i].data.misc.color.g/255., (*plg)[i].data.misc.color.b/255., 1
					//1,0,0, 1
					//);

			//for (unsigned int u=0; u< (*plg)[i].data.hulls.convex.polygon->size(); u++)
			//{
				//p.x = (*plg)[i].data.hulls.convex.polygon->points[u].x; 
				//p.y = (*plg)[i].data.hulls.convex.polygon->points[u].y; 
				//p.z = (*plg)[i].data.hulls.convex.polygon->points[u].z; 
				//msg.points.push_back(p);
			//}
			//p.x = (*plg)[i].data.hulls.convex.polygon->points[0].x; 
			//p.y = (*plg)[i].data.hulls.convex.polygon->points[0].y; 
			//p.z = (*plg)[i].data.hulls.convex.polygon->points[0].z; 
			//msg.points.push_back(p);

			//marker_vec.push_back(msg);
		//}

		//if ((int)(*plg)[i].data.hulls.convex.extended_polygon->points.size() >1) //extended convex hull marker (LINE_STRIP)
		//{
			//visualization_msgs::Marker msg = create_visualization_marker_header(
					//(*plg)[i].data.frames.global_name, ros::Time::now(),"extended_convex_hull",
					//visualization_msgs::Marker::ADD, id++, visualization_msgs::Marker::LINE_STRIP,
					//0,0,0,
					//0,0,0,1,
					//0.05,1,1,
					//(*plg)[i].data.misc.color.r/255./4, (*plg)[i].data.misc.color.g/255./4, (*plg)[i].data.misc.color.b/255./4, 1
					//);

			//for (unsigned int u=0; u< (*plg)[i].data.hulls.convex.extended_polygon->size(); u++)
			//{
				//p.x = (*plg)[i].data.hulls.convex.extended_polygon->points[u].x; 
				//p.y = (*plg)[i].data.hulls.convex.extended_polygon->points[u].y; 
				//p.z = (*plg)[i].data.hulls.convex.extended_polygon->points[u].z; 
				//msg.points.push_back(p);
			//}
			//p.x = (*plg)[i].data.hulls.convex.extended_polygon->points[0].x; 
			//p.y = (*plg)[i].data.hulls.convex.extended_polygon->points[0].y; 
			//p.z = (*plg)[i].data.hulls.convex.extended_polygon->points[0].z; 
			//msg.points.push_back(p);

			//marker_vec.push_back(msg);
		//}



		//}






		//Draw the camera projections 

		//for (size_t j=0; j< (*plg)[i].cp.size(); j++)
		//{
		//visualization_msgs::Marker msg11; //declare the msg                                              
		//msg11.header.frame_id = (*plg)[i].data.frames.global_name;
		//msg11.header.stamp = ros::Time::now();
		//msg11.ns = "raw_projection";
		//msg11.action = visualization_msgs::Marker::ADD;
		//msg11.pose.orientation.w = 1.0;
		//msg11.type = visualization_msgs::Marker::POINTS;
		//msg11.id = id++;

		//msg11.scale.x = 1; 
		//msg11.scale.y = 1; 
		//msg11.color.r = (*plg)[i].data.misc.color.r/255.;
		//msg11.color.g = (*plg)[i].data.misc.color.g/255.;
		//msg11.color.b = (*plg)[i].data.misc.color.b/255.; 
		//msg11.color.a = 1;

		//for (size_t u=0; u< (*plg)[i].cp[j].vertex_projectable.points.size(); u++)
		//{
		//p.x = (*plg)[i].cp[j].vertex_projectable.points[u].x; 
		//p.y = (*plg)[i].cp[j].vertex_projectable.points[u].y; 
		//p.z = (*plg)[i].cp[j].vertex_projectable.points[u].z; 
		////p.rgb = (*plg)[i].cp[j].vertex_projectable.points[u].rgb; 

		//msg11.points.push_back(p);

		//uint32_t rgb = (uint32_t)(*plg)[i].cp[j].vertex_projectable.points[u].rgb; 
		//uint8_t r = (rgb >> 16) & 0x0000ff;
		//uint8_t g = (rgb >> 8)  & 0x0000ff;
		//uint8_t b = (rgb)     & 0x0000ff;


		//color.r = (float)r/255.0;
		//color.g = (float)g/255.0;
		//color.b = (float)b/255.0;
		//color.r = 1.0;
		//color.g = 0.0;
		//color.b = 0.0;
		//color.a =1;

		//msg11.colors.push_back(color);

		//}

		//marker_vec.push_back(msg11);
		//}
	}

//	marker_array_msg->set_markers_vec(marker_vec);	
}




#endif
/**
 *@}
 */

