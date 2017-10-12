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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>

#include "std_msgs/String.h"
// #include "humanPose.h" //this is for the msg defined
#include "humanPoses.h"

#include "TrajectoryObservation.h"

geometry_msgs::PoseStamped leader_pose;
geometry_msgs::PoseStamped leader_goal[4];
geometry_msgs::PoseStamped robot_goal;
geometry_msgs::PoseStamped next_pose;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
move_base_msgs::MoveBaseGoal goal;
nav_msgs::Path leader_path;
geometry_msgs::Twist cmd_vel;
trajectory_simulator::TrajectoryObservation candidate_; //number of markers

tf::TransformListener* listener = NULL;
tf::StampedTransform transform;
geometry_msgs::PoseStamped source_pose;
geometry_msgs::PoseStamped target_pose;

ros::Subscriber pose_subscriber;
ros::Subscriber ar_pose_sub;
ros::Subscriber dyn_objects_subscriber;
ros::Subscriber leader_goal_subscriber;
ros::Subscriber robot_goal_subscriber;
ros::Subscriber robot_pose_subscriber;

ros::Publisher pathPublisher;
ros::Publisher robot_cmd_vel;
ros::Publisher next_posePublisher;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int num_markers;
int marker_id, marker_type;
uint leader_id = 99; //must be different from any id at initialization
double goals_distance = 99;
double robot2goal_distance, leader2goal_distance;
bool path_initialized = false;
bool send_goal = true;
bool start_to_follow = false;
bool leader_found = false;
double dist, theta, theta_max = M_PI, dmax = 15.0, dmin = 1.5;
double d , tnow, tlast;

void target_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
void ar_pose_callback(const ar_pose::ARMarkers::ConstPtr& markers);
void dyn_objects_callback(const trajectory_simulator::TrajectoryObservation::ConstPtr & dyn_objects);
void leader_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg);

double euclidean_dist(double x1,double y1,double x2, double y2){
  double dist2 = ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
  return  (sqrt(dist2));
}


#endif // HUMAN_PROC_H

