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
/*! \brief Visualization of leader classification
 *
 *  this program receives a message from a matlab code that 
 *  contains a position of a target and its classification.
 *  this information is used to create a marker message so
 *  it can be visualized in Rviz
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf/transform_listener.h"
#include "mtt/TargetList.h"
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

void drawCallback(const geometry_msgs::Pose& input)
{
  visualization_msgs::Marker marker;
  
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "quality";
  marker.id = input.orientation.x;

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
  if(input.position.z == -1){
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
  }
  else if (input.position.z == 1){
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
  }
      
  marker.color.b = 0.2f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(2);

  // Publish the marker
  marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "class_view");
  ros::NodeHandle n;

  ros::Subscriber sub_draw = n.subscribe("/pose_to_draw", 1000, drawCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("/leader_quality", 1);

  ros::spin();

  return 0;
}
