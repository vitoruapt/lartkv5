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
/*! \brief Tag good/bad leader in bag files
 *
 *  this program receives input from the keyboard (arrow keys)
 *  and when this happens, a message called /timetag is created
 *  and published. this message should be subscribed by a bag
 *  recorder (together with all the messages of interest) so the 
 *  result is a bag equal to the original + a tag.
 *  this tag signals when a good leader becames a bad leader and
 *  this is used by the extract_features code together with other
 *  measurements from each target.
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include "mtt/TargetList.h"
// #include "mtt/TargetListPC.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

mtt::TargetList targetList;
mtt::Target target;
visualization_msgs::Marker arrow_marker;
visualization_msgs::MarkerArray arrow_markerList;
visualization_msgs::Marker ids_marker;
visualization_msgs::MarkerArray ids_markerList;

ros::Publisher targets_pub;
ros::Publisher arrows_pub;
ros::Publisher ids_pub; 

uint interest_id = 90;

void targetsCallback(const mtt::TargetList& list){
  targetList.Targets.clear();
  for(uint i = 0; i < list.Targets.size(); i++){
    if (list.Targets[i].id == interest_id){
      target = list.Targets[i];
      targetList.Targets.push_back(target);
    }
  }
  targets_pub.publish(targetList);
}

void arrowsCallback(const visualization_msgs::MarkerArray &msg){
  arrow_markerList.markers.clear();
  for(uint i = 0; i < msg.markers.size(); i++){
    if(msg.markers[i].id == interest_id){
      arrow_marker = msg.markers[i];
      arrow_markerList.markers.push_back(arrow_marker);
    }
  }
  arrows_pub.publish(arrow_markerList);
}

void idsCallback(const visualization_msgs::MarkerArray &msg){
  ids_markerList.markers.clear();
  for(uint i = 0; i < msg.markers.size(); i++){
    if(msg.markers[i].id == interest_id){
      ids_marker = msg.markers[i];
      ids_markerList.markers.push_back(ids_marker);
    }
  }
  ids_pub.publish(ids_markerList);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_filter");
  ros::NodeHandle n;
  
  ros::Rate loop_rate(10);

  ros::Subscriber sub_targets = n.subscribe("/targetsF", 1000, targetsCallback);
  ros::Subscriber sub_arrows = n.subscribe("/arrowsF", 1000, arrowsCallback);
  ros::Subscriber sub_ids = n.subscribe("/idsF", 1000, idsCallback);
  
  targets_pub = n.advertise<mtt::TargetList>("/targets", 100);
  arrows_pub = n.advertise<visualization_msgs::MarkerArray>("/arrows", 100);
  ids_pub = n.advertise<visualization_msgs::MarkerArray>("/ids", 100);
  
  while (ros::ok())
  {      
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return(0);
}



