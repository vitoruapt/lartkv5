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
 * @brief Holds the c_polygon_primitive basic methods
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_CPP_
#define _polygon_primitive_CPP_


#include "polygon_primitive.h"

/**
 * @brief Constructor. Allocates space for the required objecs.
 *
 * @param name The name to use as reference to the polygon
 * @param node 
 * @param r	the polygon r color
 * @param g the polygon g color
 * @param b the polygon b color
 */
c_polygon_primitive::c_polygon_primitive(ros::NodeHandle *node, const char* name,  unsigned char r, unsigned char g, unsigned char b)
{

	rosnode = node;
	grow_number = 0;
	allocate_space();
	set_names(name);

	//Set the name of the local reference system
	std::string tmp_str(data.misc.name); 
	std::string tmp_str1("/"); 
	data.frames.local_name = tmp_str1 + tmp_str; 

	data.misc.color.r = r;
	data.misc.color.g = g;
	data.misc.color.b = b;

	//set default values for area and solidity
	data.hulls.convex.area = NAN;
	data.hulls.convex.solidity = NAN;
	data.hulls.concave.area = NAN;
	data.hulls.concave.solidity = NAN;

	//ROS_INFO("Starting a new polygon class named %s",data.misc.name);
};

/**
 * @brief Destructor. Frees the space of objects.
 */
c_polygon_primitive::~c_polygon_primitive()
{
	//reset all shared ptr to free
	pointclouds.all.reset();
	pointclouds.projected.reset();
	pointclouds.additional.reset();
	data.hulls.convex.polygon.reset();
	data.hulls.convex.extended_polygon.reset();
	data.hulls.concave.polygon.reset();
	data.hulls.concave.extended_polygon.reset();
	pointclouds.growed.reset();
	pointclouds.tmp.reset();
	data.planes.current.reset();;
	data.planes.previous.reset();;
};


#endif
/**
 *@}
 */      
