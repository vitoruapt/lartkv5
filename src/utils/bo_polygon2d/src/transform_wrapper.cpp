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
 * @addtogroup transform_wrapper
 * @{
 * @file transform_wrapper.cpp
 * @brief Provides transformations from 3D point clouds and a support plane to a
 * 2D polygon.
 */

#ifndef _TRANSFORM_WRAPPER_CPP
#define _TRANSFORM_WRAPPER_CPP

#include <bo_polygon2d/transform_wrapper.h>


class_transform_wrapper::class_transform_wrapper(void){is_null_transform=false;};
class_transform_wrapper::~class_transform_wrapper(void){};

int class_transform_wrapper::set_transform(tf::Transform* st)
{
	transform.setRotation(st->getRotation());	
	transform.setOrigin(st->getOrigin());
	return 1;
}


int class_transform_wrapper::set_null_transform(void){is_null_transform=true;return 1;};


int class_transform_wrapper::transform_global_to_local(pcl::PointCloud<pcl::PointXYZ>* pc_global, pcl::PointCloud<pcl::PointXYZ>* pc_local)
{
	if (!is_null_transform)
	{
		pcl_ros::transformPointCloud(*pc_global, *pc_local, transform); 
	}
	else
	{
		(*pc_local)= (*pc_global);
	}
	return 1;
}

int class_transform_wrapper::transform_local_to_global(pcl::PointCloud<pcl::PointXYZ>* pc_local, pcl::PointCloud<pcl::PointXYZ>* pc_global)
{
	if (!is_null_transform)
	{
		pcl_ros::transformPointCloud(*pc_local, *pc_global, transform.inverse()); 
	}
	else
	{
		(*pc_global)= (*pc_local);
	}
	return 1;
}


#endif
/**
 *@}
 */      


