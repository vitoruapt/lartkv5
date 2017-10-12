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
\file
\brief Egomotion from lidar main header
*/

#ifndef _LIDAR_EGOMOTION_H_
#define _LIDAR_EGOMOTION_H_

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <csm/csm_all.h>//needed for the PLICP algorithm, external
#include <csm/laser_data.h>//needed for the PLICP algorithm, external
#include "psm/polar_match.h"//neeeded for the psm algorithm, internal
#include "mbicp/MbICP2.h"//neeeded for the psm algorithm, internal

// #include <atlascar_base/AtlascarStatus.h>
// #include <atlascar_base/AtlascarVelocityStatus.h>

#include "motion_model.h"
#include "plplot_graph.h"
#include "types_declaration.h"

using namespace std;

void CleanZone(ray_definition*rays,double steering_angle);
void SetDefaultConfiguration(void);
void ConfigurePLICP(struct sm_params*params,LDP ref,LDP sens);
pcl::PointCloud<pcl::PointXYZ> wrapper_Laserscan2PointCloud(const sensor_msgs::LaserScan& scan);
void laser_2_Handler(const sensor_msgs::LaserScan& scan);
void laser_3_Handler(const sensor_msgs::LaserScan& scan);
// void plcStatusHandler(const atlascar_base::AtlascarStatus& scan);
// void velocityStatusHandler(const atlascar_base::AtlascarVelocityStatus& scan);

void RaysToLDP(ray_definition*src,LDP dst,bool test_angle=0,double angle=M_PI/2.5);
void RaysToLDP(ray_config_t*cfg,ray_measurment_t*src,LDP dst,bool test_angle=0,double angle=M_PI/2.5);
void RaysToPMScan(ray_config_t*cfg,ray_measurment_t*src,PMScan*dst);
void RaysToPMScan(ray_definition*src,PMScan*dst);
int RaysToMbICP(ray_config_t*cfg,ray_measurment_t*src,Tpfp*dst,bool test_angle=0,double angle=M_PI/2.2);
int RaysToMbICP(ray_definition*src,Tpfp*dst);
void ConvertEstimatedToMeasurment(double vl,double dir,float*dx,float*dy,float*dtheta,double dt,double l,double bwa);
void CreateMeasurementFromDisplacement(double dx,double dy,double dtheta,double z[2],double dt,double l,double bwa);

// extern "C" {
// 	extern LDP ld_alloc_new(int nrays);
// }
#endif

