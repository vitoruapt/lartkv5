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
\brief Generic types declaration for use in the lidar_egomotion algorithm
*/

#ifndef _TYPES_DECLARATION_H_
#define _TYPES_DECLARATION_H_

#include <iostream>
#include <iomanip>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <vector>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cxcore.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define deg2rad(a) (a*M_PI/180.)
#define rad2deg(a) (a*180./M_PI)

using namespace std;
using namespace boost;

typedef struct
{
	double theta;
	CvSeq*range;
	CvMemStorage*range_storage;
}ray_measurment_t;

typedef struct
{
	double angular_resolution;
	double minimum_delta;
	int max_points_per_ray;
	int num_rays;
	double x,y,t;//pose of this scan
	double timestamp;//timestamp of this scan
}ray_config_t;

typedef struct
{
	ray_config_t cfg; 
	ray_measurment_t*dt;
}ray_definition;

typedef struct
{
	CvSeq*data;
	CvMemStorage*data_storage;
}ray_history;

typedef struct
{
	double fps[100];
	unsigned int position;
	unsigned int current_size;
}t_fps;

/**
\brief This structure contains global flags parameters
*/
typedef struct
{
	///first point of the scan
	bool fp_s;
	///first scan
	bool fi;
	///display raw only
	bool raw_only;
}t_flag;

typedef struct s_pose
{
	s_pose()
	{
		x=0;
		y=0;
		phi=0;
		orientation=0;
		vl=0;
		vr=0;
		m_phi=0;
		m_vl=0;
		timestamp=0;
	}
	
	double x, y, phi;
	
	double orientation;
	double vl,vr;
	
	double m_phi,m_vl;
	
	double timestamp;
	
	friend ostream& operator<< (ostream &o, const s_pose &i)
	{
		return o << setprecision(4) << fixed 
		<<i.x<<" "<<i.y<<" "<<i.phi<<" "<<i.orientation<<" "<<i.vl<<" "<<0/*i.vr*/<<" "<<i.m_phi<<" "<<i.m_vl<<" "<<i.timestamp;
	}
	
} t_pose;

typedef boost::shared_ptr<t_pose> t_posePtr;

double max_orientation(const std::vector<t_posePtr>& vec);
double min_orientation(const std::vector<t_posePtr>& vec);
double max_x(const std::vector<t_posePtr>& vec);
double min_x(const std::vector<t_posePtr>& vec);
double max_y(const std::vector<t_posePtr>& vec);
double min_y(const std::vector<t_posePtr>& vec);

double get_m_phi(t_pose&i);
double get_m_vl(t_pose&i);
double get_phi(t_pose&i);
double get_orientation(t_pose&i);

double get_vl(t_pose&i);

void ClearRays(ray_measurment_t*rays,ray_config_t*ray_config);
void ClearRays(ray_definition*src);
int Theta2Index(double theta,ray_config_t*config);
void RemoveOverlappingPoints(ray_config_t*config,ray_measurment_t*rays);
void RemoveOverlappingPoints(ray_definition*src);
void Pointcloud2Rays(pcl::PointCloud<pcl::PointXYZ>& point_cloud,ray_config_t*config,ray_measurment_t*rays);
void PointCloud2Ray(pcl::PointCloud<pcl::PointXYZ>& point_cloud,ray_definition*dst);
void InitRayDefinition(ray_definition*src);
void InitRayHistory(ray_history*h_rays);
void AddToHistory(ray_definition*rays,ray_history*h_rays);
double get_fps(double dt,t_fps*acc);
void CopyRays(ray_config_t*cfg,ray_measurment_t*src,ray_measurment_t*dst);
void CopyRays(ray_definition*src,ray_definition*dst);

#endif
