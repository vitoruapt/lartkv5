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
#ifndef HUMAN_PROC_H
#define HUMAN_PROC_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>
// #include <ar_pose/object.h>

#include "std_msgs/String.h"

//#include "social_filter/humanPose.h" //this is for the msg defined
//#include "social_filter/humanPoses.h"

// #include "trajectory_simulator/TrajectoryObservation.h"
#include "leader_follower/TrajectoryObservation.h"

//social_filter::humanPose ped;
//social_filter::humanPoses list_ped;
// trajectory_simulator::TrajectoryObservation ghmm_wrapper;
ros::Publisher trajectory_pub;
int marker_id, marker_type;
ros::Duration duration_since_detection;
int list_size = 10;

struct ar_management{
//   int type;
  ros::Time detect_time;
  trajectory_simulator::TrajectoryObservation ghmm_wrapper;
};

ar_management markers_list[10];

void manage_list(int marker_id);

double euclidean_dist(double x1,double y1,double x2, double y2){
  double dist2 = ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
  return  (sqrt(dist2));
}

class ar_pose_reader
{ 
public:
  ar_pose_reader(std_msgs::String topic_name, ros::NodeHandle* n);
//   void ar_pose_callback(const ar_pose::ARMarkers::ConstPtr&);//for single marker
  void ar_pose_callback(const ar_pose::ARMarkers::ConstPtr&); //for multi markers
  void init();
  social_filter::humanPose getPose();
  ros::NodeHandle *local_n;
  ros::Subscriber ar_pose_sub;
  //social_filter::humanPose local_ped;
  std_msgs::String name;
  int num_markers;
  
  tf::TransformListener listener;
  geometry_msgs::PoseStamped source_pose;
  geometry_msgs::PoseStamped target_pose;
};

class ar_humanProc
{
 public:
  
  ar_humanProc(); 
  ~ar_humanProc();  
  void init();
  void pub();
//   void broadcast();
  std::vector<double> vec_x;
  std::vector<double> vec_y;
  std::vector<double> vec_theta;
  std::vector<ar_pose_reader*> readers;

 protected:
  ros::NodeHandle n;
  ros::Publisher pose_pub;
//   social_filter::humanPose ped;
//   social_filter::humanPoses list_ped;
//   ar_pose::ARMarker ar_pose_msg; //for single marker
  ar_pose::ARMarkers ar_pose_msg; //for multi markers
  unsigned int init_index;
  unsigned int NoH;
};


#endif // HUMAN_PROC_H

