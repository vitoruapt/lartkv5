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
#ifndef _REPRESENTATION_RVIZ_H_
#define _REPRESENTATION_RVIZ_H_

#include <navigability_map/navigability_map.h>


/**
 * \brief  Declarion functions and class to handle the visualization markers of the Class Navigability_Map.  
 * \file representation_rviz.h
 * \author Diogo Matos
 * \date June 2013
 */


/**
 * \class Markers
 * Class to handle the visualization markers 
 * \author Jorge Almeida
 * 
 */

class Markers
{
	public:		
		void update(visualization_msgs::Marker& marker)
		{
			for(uint i=0;i<markers.size();++i)
				if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
				{
					markers.erase(markers.begin()+i);
					break;
				}
				
			markers.push_back(marker);
		}
		
		void decrement(void)
		{
			for(uint i=0;i<markers.size();++i)
			{
				switch(markers[i].action)
				{
					case visualization_msgs::Marker::ADD:
						markers[i].action = visualization_msgs::Marker::DELETE;
						break;
					case visualization_msgs::Marker::DELETE:
						markers[i].action = -1;
						break;
				}
			}
		}
		
		void clean(void)
		{
			vector<visualization_msgs::Marker> new_markers;

			for(uint i=0;i<markers.size();++i)
				if(markers[i].action!=-1)
					new_markers.push_back(markers[i]);
				
			markers=new_markers;
		}

		vector<visualization_msgs::Marker> getOutgoingMarkers(void)
		{
			vector<visualization_msgs::Marker> m_out(markers);
			return markers;
		}
		
	private:
		
		vector<visualization_msgs::Marker> markers;
};

/**
* \brief Function to generate marker for the visualization of the Elevation Map on the Class Navigability_Map.
*
* \param mav_map Class Navigability_Map
* \param grid_frame frame_id to publish the markers
* \return vector<visualization_msgs::Marker>
*/

vector<visualization_msgs::Marker> create_Markers_ElavationMap(Navigability_Map& nav_map, std::string grid_frame);

/**
* \brief Function to generate marker for the visualization of the Normals on the Class Navigability_Map.
*
* \param mav_map Class Navigability_Map
* \param grid_frame frame_id to publish the markers
* \return visualization_msgs::Marker
*/

visualization_msgs::Marker create_Markers_Normals(Navigability_Map& nav_map, std::string grid_frame);

/**
* \brief Function to generate marker for the visualization of the Accessibility Map on the Class Navigability_Map.
*
* \param mav_map Class Navigability_Map
* \param grid_frame frame_id to publish the markers
* \return vector<visualization_msgs::Marker>
*/

vector<visualization_msgs::Marker> create_Markers_AccessibilityMap(Navigability_Map& nav_map, std::string grid_frame);

/// _______________________
///						    |
/// Ground truth functions |
///________________________|
visualization_msgs::Marker create_Markers_Polygon(Navigability_Map& nav_map, std::string grid_frame);
visualization_msgs::Marker create_Markers_Obstacle(Navigability_Map& nav_map, std::string grid_frame);


#endif
