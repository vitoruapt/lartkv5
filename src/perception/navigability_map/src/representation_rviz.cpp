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
* \brief Visualization on rviz related functions.
* \file representation_rviz.cpp
* \author Diogo Matos
* \date June 2013
*/

#include "representation_rviz.h"

vector<visualization_msgs::Marker> create_Markers_ElavationMap(Navigability_Map& nav_map, std::string grid_frame)
{
	static Markers marker_list;
	grid_nodePtr node_data;
	double Zmin=nav_map.Zmin_filter;
	double Zmax=nav_map.Zmax_filter;
	int colorrange=nav_map.colorrange;
	
	//Reduce the elements status, ADD to REMOVE and REMOVE to delete
	marker_list.decrement();
	
	// Create a colormap
	
	class_colormap colormap_positive("jet",colorrange, 1, false);
	class_colormap colormap_negative("pink",colorrange, 1, false);
	
	/***** Elevation Cell *****/
	visualization_msgs::Marker marker_cell;
	
	marker_cell.header.frame_id = grid_frame;
    marker_cell.header.stamp = ros::Time::now();;
	marker_cell.ns = "elvation_cell";
	marker_cell.action = visualization_msgs::Marker::ADD;
	marker_cell.type = visualization_msgs::Marker::CUBE;
	marker_cell.pose.orientation.x = 0;	marker_cell.pose.orientation.y = 0.0;	marker_cell.pose.orientation.z = 0;	marker_cell.pose.orientation.w = 1;
	
	marker_cell.scale.x = nav_map.Sx;
	marker_cell.scale.y = nav_map.Sy;

	geometry_msgs::Point p;
	
	/***** Confidence Cell *****/
	visualization_msgs::Marker marker_text;
	
	marker_text.header.frame_id = grid_frame;
    marker_text.header.stamp = ros::Time::now();;
	marker_text.ns = "confidence_cell";
	marker_text.action = visualization_msgs::Marker::ADD;
	marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_text.scale.x = 0.07;
	marker_text.scale.y = 0.07;
	marker_text.scale.z = 0.07;
	marker_text.color.a = 1.0;    
    marker_text.color.r = 0.0;   
    marker_text.color.g = 0.0;
    marker_text.color.b = 0.0;
	
	
	uint counter=0;
	for(uint row=0;row<nav_map.grid.rows();row++)
	{
		//X posistion
		p.x = row*nav_map.Sx+nav_map.Sx/2;
		
		for(uint col=0;col<nav_map.grid.cols();col++)
		{
			
			node_data = nav_map.grid(row,col);
			
			if (node_data->num_points>0 || node_data->interpolate_z_data) //node is occupied
			{
				//id
				marker_cell.id = counter;
				
				//Y position
				p.y = col*nav_map.Sy-nav_map.CARaxis_col*nav_map.Sy;
				
				//Scale and Color
				
				p.z= node_data->Zmed/2;
				marker_cell.scale.z = node_data->Zmed;
				
				//color
				if (node_data->Zmed>=0 )
					marker_cell.color = colormap_positive.color((colorrange*node_data->Zmed)/Zmax);
				else
					marker_cell.color =colormap_positive.color((colorrange*abs(node_data->Zmed))/abs(Zmin));				
					
				//set cell pos
				marker_cell.pose.position=p;
				
				//update marker
				marker_list.update(marker_cell);
					
				/****** Text Confidence cell *****/

				//id
				marker_text.id = counter;
				
				//position
				marker_text.pose.position.x = p.x;
				marker_text.pose.position.y = p.y;
				if (p.z>=0)
					marker_text.pose.position.z = node_data->Zmed+0.05; 
				else
					marker_text.pose.position.z = 0.05; 
				
				//text
				boost::format fm("%d");
				fm % node_data->Z_confidence;
// 				fm % node_data->angle_confidence_y;
// 				fm % node_data->Zmed;
				marker_text.text = fm.str();
				
				//update marker
// 				marker_list.update(marker_text);

				
				
			}	
			counter++;
		}
	}

	//Remove markers that should not be transmitted
	marker_list.clean();
	
	//Clean the marker_vector and put new markers in it;
	return marker_list.getOutgoingMarkers();
}



visualization_msgs::Marker create_Markers_Normals(Navigability_Map& nav_map, std::string grid_frame)
{
	grid_nodePtr node_data;
	double px_aux=0;
	double py_aux=0;
	double pz_aux=0;
	//publish a marker to rviz with the normals
	visualization_msgs::Marker mk; //declare the array
	
	geometry_msgs::Point p;
	std_msgs::ColorRGBA color;

	mk.header.frame_id = grid_frame; mk.header.stamp = ros::Time::now();	mk.ns = "normals";
	mk.action = visualization_msgs::Marker::ADD;	mk.type = visualization_msgs::Marker::LINE_LIST; mk.id = 0;
	mk.pose.position.x = 0; mk.pose.position.y = 0; mk.pose.position.z = 0;
	mk.pose.orientation.x = 0;	mk.pose.orientation.y = 0;	mk.pose.orientation.z = 0;	mk.pose.orientation.w = 1;
	mk.scale.x = 0.01; mk.scale.y = 0.01; mk.scale.z = 0;
	mk.color.r = 0.7; mk.color.g = 0; mk.color.b = 0; mk.color.a = 1;

	for(uint row=0;row<nav_map.grid.rows();row++)
	{
		for(uint col=0;col<nav_map.grid.cols();col++)
		{
			node_data = nav_map.grid(row,col);
			
			if (node_data->has_normal) //node has_normal 
			{
				//X posistion
				p.x = row*nav_map.Sx+nav_map.Sx/2;
				px_aux = p.x;
				
				//Y position
				p.y = col*nav_map.Sy-nav_map.CARaxis_col*nav_map.Sy;
				py_aux = p.y;
				
				//Z posistion
				p.z= node_data->Zmed;
				if (p.z>=0)
					pz_aux=p.z; 
				else
				{
					p.z =0;
					pz_aux = 0; 
				}
					
				mk.points.push_back(p);	

				double rat=0.5;

				p.x = px_aux + (cos(node_data->med_angle_X))*rat;
				p.y = py_aux + (cos(node_data->med_angle_Y))*rat;
				p.z = pz_aux + (cos(node_data->med_angle_Z))*rat;
		
				mk.points.push_back(p);	
			}
		}
	}
	
	return mk;
}

vector<visualization_msgs::Marker> create_Markers_AccessibilityMap(Navigability_Map& nav_map, std::string grid_frame)
{
	static Markers marker_list;
	grid_nodePtr node_data;

	int colorrange=nav_map.colorrange;
	
	//Reduce the elements status, ADD to REMOVE and REMOVE to delete
	marker_list.decrement();
	
	// Create a colormap
	
	class_colormap colormap_positive("jet",colorrange, 1, false);
	
	/***** _Accessibility Cell *****/
	visualization_msgs::Marker marker_cell;
	
	marker_cell.header.frame_id = grid_frame;
    marker_cell.header.stamp = ros::Time::now();;
	marker_cell.ns = "accessibility_cell";
	marker_cell.action = visualization_msgs::Marker::ADD;
	marker_cell.type = visualization_msgs::Marker::CUBE;
	marker_cell.pose.orientation.x = 0;	marker_cell.pose.orientation.y = 0.0;	marker_cell.pose.orientation.z = 0;	marker_cell.pose.orientation.w = 1;
	
	marker_cell.scale.x = nav_map.Sx;
	marker_cell.scale.y = nav_map.Sy;

	geometry_msgs::Point p;
	
	
	/***** debug Cell *****/
	visualization_msgs::Marker marker_text;
	
	marker_text.header.frame_id = grid_frame;
    marker_text.header.stamp = ros::Time::now();;
	marker_text.ns = "debug_cell";
	marker_text.action = visualization_msgs::Marker::ADD;
	marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_text.scale.x = 0.07;
	marker_text.scale.y = 0.07;
	marker_text.scale.z = 0.07;
	marker_text.color.a = 1.0;    
    marker_text.color.r = 0.0;   
    marker_text.color.g = 0.0;
    marker_text.color.b = 0.0;
	
	
	uint counter=0;
	for(uint row=0;row<nav_map.grid.rows();row++)
	{
		//X posistion
		p.x = row*nav_map.Sx+nav_map.Sx/2;
		
		for(uint col=0;col<nav_map.grid.cols();col++)
		{
			
			node_data = nav_map.grid(row,col);
			
			if (node_data->num_points>0 || node_data->interpolate_z_data) //node is occupied
			{
				//id
				marker_cell.id = counter;
				
				//Y position
				p.y = col*nav_map.Sy-nav_map.CARaxis_col*nav_map.Sy;
				
// 				Scale and Color
				if (nav_map.debug_accessibility==1)
					p.z=(1-node_data->z_accessibility)/2;
				else if (nav_map.debug_accessibility==2)
					p.z=(1-node_data->angleX_accessibility)/2;
				else if (nav_map.debug_accessibility==3)
					p.z=(1-node_data->angleY_accessibility)/2;
				else if (nav_map.debug_accessibility==4)
					p.z=(1-node_data->angleZ_accessibility)/2;
				else if (nav_map.debug_accessibility==5)
					p.z=(1-node_data->total_accessibility)/2;
				
				if (nav_map.debug_accessibility==1)
					marker_cell.scale.z =1-node_data->z_accessibility;
				else if (nav_map.debug_accessibility==2)
					marker_cell.scale.z =1-node_data->angleX_accessibility;
				else if (nav_map.debug_accessibility==3)
					marker_cell.scale.z =1-node_data->angleY_accessibility;
				else if (nav_map.debug_accessibility==4)
					marker_cell.scale.z =1-node_data->angleZ_accessibility;
				else if (nav_map.debug_accessibility==5)
					marker_cell.scale.z =1-node_data->total_accessibility;
				
				//color
				if (nav_map.debug_accessibility==1)
					marker_cell.color = colormap_positive.color(colorrange*(1-node_data->z_accessibility)-1);
				else if (nav_map.debug_accessibility==2)
					marker_cell.color = colormap_positive.color(colorrange*(1-node_data->angleX_accessibility)-1);
				else if (nav_map.debug_accessibility==3)
					marker_cell.color = colormap_positive.color(colorrange*(1-node_data->angleY_accessibility)-1);
				else if (nav_map.debug_accessibility==4)
					marker_cell.color = colormap_positive.color(colorrange*(1-node_data->angleZ_accessibility)-1);
				else if (nav_map.debug_accessibility==5)
					marker_cell.color = colormap_positive.color(colorrange*(1-node_data->total_accessibility)-1);

				
				//set cell pos
				marker_cell.pose.position=p;
				
				//update marker
				marker_list.update(marker_cell);
				
				
				/******  debug Confidence cell *****/
// 				id
				marker_text.id = counter;
				
				//position
				marker_text.pose.position.x = p.x;
				marker_text.pose.position.y = p.y;
				
				if (nav_map.debug_accessibility==1)
					marker_text.pose.position.z = (1-node_data->z_accessibility)+0.1; 
				else if (nav_map.debug_accessibility==2)
					marker_text.pose.position.z = (1-node_data->angleX_accessibility)+0.1; 
				else if (nav_map.debug_accessibility==3)
					marker_text.pose.position.z = (1-node_data->angleY_accessibility)+0.1; 
				else if (nav_map.debug_accessibility==4)
					marker_text.pose.position.z = (1-node_data->angleZ_accessibility)+0.1; 
				else if (nav_map.debug_accessibility==5)
					marker_text.pose.position.z = (1-node_data->total_accessibility)+0.1;
				
				//text
				boost::format fm("%d");
				if (nav_map.debug_accessibility==1)
					fm % node_data->z_accessibility;
				else if (nav_map.debug_accessibility==2)
					fm % node_data->angleX_accessibility;
				else if (nav_map.debug_accessibility==3)
					fm % node_data->angleY_accessibility;
				else if (nav_map.debug_accessibility==4)
					fm % node_data->angleZ_accessibility;
				else if (nav_map.debug_accessibility==5)
					fm % node_data->total_accessibility;
			
				marker_text.text = fm.str();
				//update marker
// 				marker_list.update(marker_text);
				
			}
			counter++;
		}	
	}

	//Remove markers that should not be transmitted
	marker_list.clean();
	
	//Clean the marker_vector and put new markers in it;
	return marker_list.getOutgoingMarkers();	
}



visualization_msgs::Marker create_Markers_Polygon(Navigability_Map& nav_map, std::string grid_frame)
{
	//publish a marker to rviz with the polygon
	visualization_msgs::Marker mk; //declare the array
	
	std_msgs::ColorRGBA color;

	mk.header.frame_id = grid_frame; mk.header.stamp = ros::Time::now();	mk.ns = "polygon";
	mk.action = visualization_msgs::Marker::ADD;	mk.type = visualization_msgs::Marker::LINE_STRIP; mk.id = 0;
	mk.pose.position.x = 0; mk.pose.position.y = 0; mk.pose.position.z = 0;
	mk.pose.orientation.x = 0;	mk.pose.orientation.y = 0;	mk.pose.orientation.z = 0;	mk.pose.orientation.w = 1;
	mk.scale.x = 0.08; mk.scale.y = 0.01; mk.scale.z = 0;
	mk.color.r = 0; mk.color.g = 1; mk.color.b = 0; mk.color.a = 1;

	
	for (uint i=0; i<nav_map.polygon_points.size(); i++)
		mk.points.push_back(nav_map.polygon_points[i]);	
	
	nav_map.polygon_points.erase(nav_map.polygon_points.begin(),nav_map.polygon_points.end());
	return mk;
}

visualization_msgs::Marker create_Markers_Obstacle(Navigability_Map& nav_map, std::string grid_frame)
{
	//publish a marker to rviz with the polygon
	visualization_msgs::Marker mk; //declare the array
	
	std_msgs::ColorRGBA color;

	mk.header.frame_id = grid_frame; mk.header.stamp = ros::Time::now();	mk.ns = "polygon_obstacle";
	mk.action = visualization_msgs::Marker::ADD;	mk.type = visualization_msgs::Marker::LINE_STRIP; mk.id = 0;
	mk.pose.position.x = 0; mk.pose.position.y = 0; mk.pose.position.z = 0;
	mk.pose.orientation.x = 0;	mk.pose.orientation.y = 0;	mk.pose.orientation.z = 0;	mk.pose.orientation.w = 1;
	mk.scale.x = 0.08; mk.scale.y = 0.01; mk.scale.z = 0;
	mk.color.r = 1; mk.color.g = 0; mk.color.b = 0; mk.color.a = 1;

	
	for (uint i=0; i<nav_map.obstacle_points.size(); i++)
		mk.points.push_back(nav_map.obstacle_points[i]);	
	
	nav_map.obstacle_points.erase(nav_map.obstacle_points.begin(),nav_map.obstacle_points.end());
	return mk;
}
