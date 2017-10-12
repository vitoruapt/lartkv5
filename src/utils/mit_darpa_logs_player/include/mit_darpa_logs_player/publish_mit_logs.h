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
 * @addtogroup mit_logs
 * @file 
 * @brief header file for publish_mit_logs.cpp
 *@{
 */
#ifndef _READ_VELODYNE_H_
#define _READ_VELODYNE_H_

//####################################################################
// Includes:
//####################################################################

#include <ros/ros.h>
//#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_types.h>
//#include <ros/ros.h>
// PCL specific includes
 #include <pcl/conversions.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>

#include <signal.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "config_util.h"
#include "eventlog.h"
#include "lcmtypes_velodyne_t.h"
#include "lcmtypes_image_t.h"
#include "lcmtypes_pose_t.h"
#include "small_linalg.h"
#include "velodyne.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//#include <pcl/io/pcd_io.h>

//#include "tp.h"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>  

#include "config_util.h"
#include "rotations.h"
#include "pcl_ros/transforms.h"    

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

// #include <cv_bridge/CvBridge.h>  
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>                                                           
#include <Eigen/Geometry>


#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}




#endif
/**
*@}
*/
