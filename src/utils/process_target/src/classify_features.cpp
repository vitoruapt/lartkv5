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
/*! \brief Code for feature extraction
 *
 *  extract features from targets received from MTT wrt the robot
 *  the features are printed in the command line and should be stored
 *  in a file (txt) for further processing with matlab
 *  the features will be used for training a classifier
 * 
 *  it is also responsible of publishing these features to a matlab
 *  classifier that will then output a message with the classification
 *  of the target based on its features
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf/transform_listener.h"
#include "mtt/TargetList.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "opencv/cv.h"
#include "opencv/ml.h"
#include <vector>
#include <iostream>

// using namespace std;
// using namespace cv;

CvBoost adaboost;
CvMat* sample_to_classify = 0;
CvMat* weak_responses = 0;

// static ros::Time start_time;
ros::Time time_last_msg(0);
ros::Duration time_elapsed;
ros::Duration duration_btw_msg;
ros::Publisher nfeatures_pub;
ros::Publisher marker_pub;

tf::TransformListener *p_listener;
tf::StampedTransform transform;

geometry_msgs::Twist twist;
geometry_msgs::Twist features;
geometry_msgs::PoseWithCovariance nfeatures;
visualization_msgs::MarkerArray ma;

std::list<geometry_msgs::PoseWithCovariance> matlab_list;

double target_x, target_y;
double target_vel, target_theta;
double robot_x, robot_y;
double robot_vel, robot_theta;
double position_diff, heading_diff;
double angle_to_robot;
double velocity_diff;
double lateral_disp;
double label;
uint badLeader = 0;
uint target_id = 0;
uint single_target = 0;
uint counter = 0;

void drawPose(const geometry_msgs::Pose& input, uint target, double classification)
{
  
  visualization_msgs::Marker marker;
  
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "quality";
  marker.id = target;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = input.position.x;
  marker.pose.position.y = input.position.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
 
  //good or bad leader
  if(classification == 1){
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
  }
  else if (classification == 2){
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
  }
      
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(1.0);
  
//   ROS_INFO("created marker, pushing back");
  ma.markers.push_back(marker);
//   ROS_INFO("created marker end");
  // Publish the marker
//   marker_pub.publish(marker);
}

void targetsCallback(const mtt::TargetList& list)
{
  static ros::Time start_time = ros::Time::now();
  time_elapsed = ros::Time::now() - start_time;
  matlab_list.clear();
  
  /// /// ROBOT PART //////
  //use transformations to extract robot features
//   try{
//     p_listener->lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
//     p_listener->lookupTwist("/map", "/robot_0/base_link", ros::Time(0), ros::Duration(0.5), twist);
//   }
  try{
    p_listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
    p_listener->lookupTwist("/map", "/base_link", ros::Time(0), ros::Duration(0.5), twist);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  robot_x = transform.getOrigin().x();
  robot_y = transform.getOrigin().y();
  robot_theta = tf::getYaw(transform.getRotation());
  robot_vel = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));
  
  /// /// TARGETS PART //////
  //sweeps target list and extract features
  for(uint i = 0; i < list.Targets.size(); i++){
    target_id = list.Targets[i].id;
    target_x = list.Targets[i].pose.position.x;
    target_y = list.Targets[i].pose.position.y;
    target_theta = tf::getYaw(list.Targets[i].pose.orientation);
    target_vel = sqrt(pow(list.Targets[i].velocity.linear.x,2)+
                      pow(list.Targets[i].velocity.linear.y,2));
    position_diff = sqrt(pow(robot_x - target_x,2)+
                        pow(robot_y - target_y,2));
    heading_diff = robot_theta - target_theta;
    angle_to_robot = -robot_theta + atan2(target_y - robot_y, 
                                        target_x - robot_x );
    velocity_diff = robot_vel - target_vel;
    lateral_disp = sin(angle_to_robot)*position_diff;
      
    
//     if(position_diff < 6.0 && target_vel > 0.5){
      sample_to_classify->data.fl[1] = target_vel;
      sample_to_classify->data.fl[2] = lateral_disp;
      sample_to_classify->data.fl[3] = heading_diff;
      sample_to_classify->data.fl[4] = angle_to_robot;
      sample_to_classify->data.fl[5] = position_diff;
      
//       ROS_INFO("classifying     ");
      label = adaboost.predict(sample_to_classify, 0, weak_responses );
//       ROS_INFO("label:%f, now will publish",label);
      drawPose(list.Targets[i].pose, target_id, label);
//     }
  }
      
  marker_pub.publish(ma);
  ma.markers.clear();
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_target");
  ros::NodeHandle n;
  
  tf::TransformListener listener;
  p_listener=&listener;

//   adaboost.load("trained_boost.xml");
  std::cout << "loaded file: " << argv[1] << std::endl;
  adaboost.load(argv[1]);
  sample_to_classify = cvCreateMat( 1, 6, CV_32F );
  weak_responses = cvCreateMat( 1, adaboost.get_weak_predictors()->total, CV_32F );
    
  ros::Subscriber sub = n.subscribe("/targets", 1000, targetsCallback);
//   ros::Subscriber sub_tag = n.subscribe("/timetag", 1000, tagCallback);
//   ros::Subscriber sub_draw = n.subscribe("/pose_to_draw", 1000, drawCallback);
  nfeatures_pub = n.advertise<geometry_msgs::PoseWithCovariance>("/example_topic", 1000);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("/leader_quality", 1000);

  ros::spin();

  return 0;
}
