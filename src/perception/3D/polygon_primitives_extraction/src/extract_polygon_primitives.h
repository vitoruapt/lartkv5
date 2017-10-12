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
 *@{
 * @file 
 * @brief Header of the extract polygon primitives binary
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _extract_polygon_primitives_H_
#define _extract_polygon_primitives_H_



//   ___________________________________
//   |                                 |
//   |           INCLUDES              |
//   |_________________________________| 
///Ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

///PLC includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

///Rviz includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

///Other modules include
#include <wrapper_collada/wrapper_collada.h>

///Local includes
#include "polygon_primitive.h"

//   ___________________________________
//   |                                 |
//   |            DEFINES              |
//   |_________________________________| 
#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}
#define _MAX_NUM_POLYGONS_ 270
//#define _MAX_NUM_POLYGONS_ 2

//////////////////////////////////////////////////////////
//TYPE DEFINITIONS
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//FUNCTIONS PROTOTYPES
//////////////////////////////////////////////////////////

//defined in extract_polygon_primitives_rvizmarkers.cpp
void create_publish_marker_array(std::vector<c_polygon_primitive> *plg, visualization_msgs::MarkerArray *marker_array_msg);
visualization_msgs::Marker create_visualization_marker_header(
		std::string frame_id, ros::Time stamp, std::string name,
		int action, int id, int type,
		double px, double py, double pz,
		double qx, double qy, double qz, double qw,
		double sx, double sy, double sz,
		double cr, double cg, double cb, double alpha);

//defined in extract_polygon_primitives_handlers.cpp
void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& input);
void VehiclePoseHandler(const geometry_msgs::PoseStamped& lpose);


//defined in extract_polygon_primitives_auxiliary.cpp
//void filter_uncoherent_points(pcl::PointCloud<pcl::PointNormal>::Ptr pc, double radius, pcl::PointCloud<pcl::PointNormal>::Ptr pcout);
void filter_uncoherent_points(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ> *pcout,  pcl::PointCloud<pcl::PointXYZ> *pcelim,pcl::PointCloud<pcl::Normal>::Ptr n ,double radius, float vx, float vy, float vz);
void filter_uncoherent_points(pcl::PointCloud<pcl::PointNormal>::Ptr pc, double radius, pcl::PointCloud<pcl::PointNormal>::Ptr pcout, pcl::PointCloud<pcl::PointNormal>::Ptr pcelim,float vx, float vy, float vz);
void draw_markers(int k, Eigen::Vector4f* q_avg, std::vector<Eigen::Vector4f>* qv, pcl::PointCloud<pcl::PointNormal>::Ptr pc, std::vector<int>* pointIdxRadiusSearch, Eigen::Vector4f* q_avg0, std::vector<bool>* is_fliped);
int compute_normals(pcl::PointCloud<pcl::PointXYZ>* pc_in, float vx, float vy, float vz, float radius, int K, pcl::PointCloud<pcl::Normal>* pc_out);
int save_pc_PointNormal_to_pcd(pcl::PointCloud<pcl::PointXYZ>* pc_in, pcl::PointCloud<pcl::Normal>* normals_in, std::string name);
int get_vehicle_position(ros::NodeHandle *pn, tf::TransformListener* listener, tf::StampedTransform* vehicle_tf, ros::Time* t);

//////////////////////////////////////////////////////////
//GLOBAL VARIABLES
//////////////////////////////////////////////////////////
#ifdef _extract_polygon_primitives_CPP_

#define _EXTERN_ 
unsigned char cr[16]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
unsigned char g[16]={0, 16, 32, 49, 65, 81, 97, 113, 130, 146, 162, 178, 194, 210, 227, 243}; 
unsigned char b[16]={255, 247, 239, 231, 223, 215, 206, 198, 190, 182, 174, 166, 158, 150, 142, 134}; 

#else

#define _EXTERN_ extern
extern unsigned char cr[16];
extern unsigned char g[16];
extern unsigned char b[16];

#endif

_EXTERN_ std::vector<boost::shared_ptr<ros::Publisher> > publishers;
_EXTERN_ pcl::PointCloud<pcl::PointXYZ> cloud;
_EXTERN_ ros::Time laser_ts;
_EXTERN_ int flag_msg_received;
_EXTERN_ ros::NodeHandle *pn; //The node handle
#endif
/**
*@}
*/
