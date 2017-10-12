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
 * @brief Header of the texturize polygon primitives binary
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 
 */
#ifndef _texturize_polygon_primitives_H_
#define _texturize_polygon_primitives_H_


//////////////////////////////////////////////////////////
//INCLUDES
//////////////////////////////////////////////////////////

///Ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

///PLC includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

///Rviz includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

///Other modules include
#include <wrapper_collada.h>

///Local includes
#include "polygon_primitive_with_texture.h"


//////////////////////////////////////////////////////////
//DEFINES
//////////////////////////////////////////////////////////
#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}
#define _MAX_NUM_POLYGONS_ 99
//////////////////////////////////////////////////////////
//TYPE DEFINITIONS
//////////////////////////////////////////////////////////
typedef struct
{
	cv::Mat image;
	cv_bridge::CvImagePtr cv_ptr;
	ros::Time image_ts;
	bool msg_received;
	tf::StampedTransform tf;
		
}t_cam;

//////////////////////////////////////////////////////////
//FUNCTIONS PROTOTYPES
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
//GLOBAL VARIABLES
//////////////////////////////////////////////////////////
#ifdef _texturize_polygon_primitives_CPP_

#define _EXTERN_ 

#else

#define _EXTERN_ extern

#endif

_EXTERN_ std::map<std::string, c_polygon_primitive_with_texture> pmap; 
_EXTERN_ t_cam cam_roof_fc;
_EXTERN_ t_cam cam_roof_fl;
_EXTERN_ t_cam cam_roof_fr;
_EXTERN_ t_cam cam_roof_rc;
_EXTERN_ t_cam cam_roof_fc_6mm;
_EXTERN_ tf::TransformListener* p_listener;
_EXTERN_ ros::NodeHandle* p_node;
_EXTERN_ ros::Publisher* p_markerarray_pub;
_EXTERN_ ros::Time mission_start;

#endif
/**
*@}
*/
