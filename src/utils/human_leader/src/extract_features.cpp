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
// #include "geometry_msgs/PoseWithCovariance.h"
#include "tf/transform_listener.h"
#include "mtt/TargetList.h"
// #include <visualization_msgs/Marker.h>

// static ros::Time start_time;
ros::Time time_last_msg(0);
ros::Duration time_elapsed;
// ros::Duration duration_btw_msg;
// ros::Publisher nfeatures_pub;
// ros::Publisher marker_pub;

tf::TransformListener *p_listener;
tf::StampedTransform transform;

geometry_msgs::Twist twist;
geometry_msgs::Twist features;
// geometry_msgs::PoseWithCovariance nfeatures;

// std::list<geometry_msgs::PoseWithCovariance> matlab_list;

double target_x, target_y;
double target_vel, target_theta;
double robot_x, robot_y;
double robot_vel, robot_theta;
double position_diff, heading_diff;
double angle_to_robot;
double velocity_diff;
uint leader_tag = 0;
uint target_id = 0;
uint single_target = 0;
// uint counter = 0;

void targetsCallback(const mtt::TargetList& list)
{
  //will print information that should be stored in a file
  //file format: id, good/bad tag, time, pos x, pos y, vel, theta
  //             position_diff, heading_diff, angle_to_robot, velocity_diff
  static ros::Time start_time = ros::Time::now();
  time_elapsed = ros::Time::now() - start_time;
//   matlab_list.clear();
  
  /// /// ROBOT PART //////
  //use transformations to extract robot features
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
   
  //robot output line 
  /// uncomment the following for training!
  printf("%d,%d,%.10f,%.10f,%.10f,%.10f,%.10f,0,0,0,0\n",
         -1, leader_tag, time_elapsed.toSec(),
         robot_x, robot_y, robot_vel, robot_theta); 
  
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
          
    //target output (to be used in adaboost training)
    
    // % output file format: 
    // % 1: id
    // % 2: good/bad tag
    // % 3: time
    // % 4: pos x
    // % 5: pos y
    // % 6: vel
    // % 7: theta
    // % 8: pos diff
    // % 9: head diff
    // %10: angle 2 robot 
    // %11: velocity diff 
    
    /// uncomment the following for training!
    printf("%d,%d,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",
      target_id, leader_tag, time_elapsed.toSec(),
      target_x, target_y, target_vel, target_theta,
      position_diff, heading_diff, angle_to_robot, velocity_diff);
    
//     if(position_diff < 6.0 && target_vel > 0.5){
//       //if inside boundaries (in meters)
//       //store features in a covariance struct to send to matlab
//       nfeatures.pose.position.x = target_x;
//       nfeatures.pose.position.y = target_y;
//       nfeatures.pose.position.z = target_id;
// 
//       nfeatures.covariance[0] = target_vel;
//       nfeatures.covariance[1] = velocity_diff;
//       nfeatures.covariance[2] = heading_diff;
//       nfeatures.covariance[3] = angle_to_robot;
//       nfeatures.covariance[4] = position_diff;
//       
//       matlab_list.push_back(nfeatures);
// //       counter++;
//     }
  }
//   printf("targets within range: %d\n",counter);
//   counter = 0;
  
  //check if enough time has passed 
  //and send batch of msgs to matlab
//   duration_btw_msg = ros::Time::now() - time_last_msg;
//   
//   if(duration_btw_msg.toSec() > 0.01){
//     while(!matlab_list.empty()){
//       nfeatures_pub.publish(matlab_list.front());
//       matlab_list.pop_front();
//       usleep(0.01e6); //has to sleep, otherwise matlab do not get the msg
//     }      
//     time_last_msg = ros::Time::now();
//   }
}

void tagCallback(const std_msgs::Header& tag)
{
  //ROS_INFO("tag received");
  //only works for a transition good/bad leader
  leader_tag = 1;
}

// void drawCallback(const geometry_msgs::Pose& input)
// {
//   visualization_msgs::Marker marker;
//   
//   marker.header.frame_id = "/map";
//   marker.header.stamp = ros::Time::now();
// 
//   marker.ns = "quality";
//   marker.id = 0;
// 
//   marker.type = visualization_msgs::Marker::CYLINDER;
//   marker.action = visualization_msgs::Marker::ADD;
// 
//   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//   marker.pose.position.x = input.position.x;
//   marker.pose.position.y = input.position.y;
//   marker.pose.position.z = 0;
//   marker.pose.orientation.x = 0.0;
//   marker.pose.orientation.y = 0.0;
//   marker.pose.orientation.z = 0.0;
//   marker.pose.orientation.w = 1.0;
// 
//   // Set the scale of the marker
//   marker.scale.x = 0.5;
//   marker.scale.y = 0.5;
//   marker.scale.z = 0.5;
//  
//   //good or bad leader
//   if(input.position.z == -1){
//     marker.color.r = 0.0f;
//     marker.color.g = 1.0f;
//   }
//   else if (input.position.z == 1){
//     marker.color.r = 1.0f;
//     marker.color.g = 0.0f;
//   }
//       
//   marker.color.b = 0.0f;
//   marker.color.a = 1.0;
// 
//   marker.lifetime = ros::Duration();
// 
//   // Publish the marker
//   marker_pub.publish(marker);
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_target");
  ros::NodeHandle n;
  
  tf::TransformListener listener;
  p_listener=&listener;

  //for testing purposes
//   single_target = atoi(argv[1]);
//   ROS_INFO("chosen target:%d",single_target);
    
  ros::Subscriber sub = n.subscribe("/targets", 1000, targetsCallback);
  ros::Subscriber sub_tag = n.subscribe("/timetag", 1000, tagCallback);
//   ros::Subscriber sub_draw = n.subscribe("/pose_to_draw", 1000, drawCallback);
//   nfeatures_pub = n.advertise<geometry_msgs::PoseWithCovariance>("/example_topic", 100);
//   marker_pub = n.advertise<visualization_msgs::Marker>("/leader_quality", 1);

  ros::spin();

  return 0;
}
