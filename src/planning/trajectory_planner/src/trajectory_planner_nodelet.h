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
#ifndef _trajectory_planner_nodelet_H_
#define _trajectory_planner_nodelet_H_

#include <ros/ros.h>
#include <trajectory_planner/c_trajectory.h>
#include <trajectory_planner/c_manage_trajectory.h>
#include <atlasmv_base/AtlasmvStatus.h>
#include <atlasmv_base/AtlasmvMotionCommand.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mtt/TargetListPC.h>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include "define_trajectories.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <trajectory_planner/coordinates.h>
#include <geometry_msgs/Pose.h>

// Defines
#define PFLN printf("FILE %s LINE %d\n",__FILE__, __LINE__);
#define _D_ 0.50
#define _NUM_TRAJ_ 20
#define _NUM_NODES_ 9

#define _VEHICLE_HEIGHT_TOP_ 0.66
#define _VEHICLE_HEIGHT_BOTTOM_ 0.055
#define _VEHICLE_LENGHT_FRONT_ 0.62
#define _VEHICLE_LENGHT_BACK_ 0.2
#define _VEHICLE_WIDTH_ 0.42

//namepaces
using namespace visualization_msgs;

#ifdef _trajectory_planner_nodelet_CPP_
#define _EXTERN_ 
#else
#define _EXTERN_ extern
#endif

//   ___________________________________
//   |                                 |
//   |        PROTOTYPES               |
//   |_________________________________| 
//   Defined in trajectory_planner_nodelet.cpp
void StatusMessageHandler(const atlasmv_base::AtlasmvStatus& msg);
int main(int argc, char **argv);
vector<double> set_speed_vector(c_trajectoryPtr t);
bool jump_node(double dist_init,int node, c_trajectoryPtr t);
void send_command_message(vector<double> speed_setted,int current_node, c_trajectoryPtr t);

// Global Vars
_EXTERN_ ros::NodeHandle* p_n;
_EXTERN_ tf::TransformListener *p_listener;
_EXTERN_ tf::StampedTransform transformw;
_EXTERN_ tf::StampedTransform transform_mtt;
_EXTERN_ ros::Publisher commandPublisher;
_EXTERN_ c_manage_trajectoryPtr manage_vt;

#endif
