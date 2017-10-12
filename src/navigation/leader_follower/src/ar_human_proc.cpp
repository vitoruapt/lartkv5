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
#include "ros/ros.h"
#include "ar_human_proc.h"
// #include <tf/transform_broadcaster.h>

#define DEG2RAD 0.01745

/*******
This node gets data from ar_pose and publishes structures for humanPoses.
There are two classes ar_humanProc and ar_pose_reader. 
********/

ar_pose_reader::ar_pose_reader(std_msgs::String topic_name,ros::NodeHandle* n)
{
  local_n=n;
  name.data.assign(topic_name.data);
}

void ar_pose_reader::init()
{
  //initialize fields
//   local_ped.x=0.0;
//   local_ped.y=0.0;
//   local_ped.theta= 0.0; //in degrees
//   local_ped.linear_velocity=0.0;
//   local_ped.angular_velocity=0.0;
  
  //subscribe to ar_pose messages
  ar_pose_sub = local_n->subscribe(name.data.c_str(), 15, &ar_pose_reader::ar_pose_callback,this);
  
  for(int i = 0; i < list_size; i++){
// 	markers_list[i].type = 0;
        markers_list[i].ghmm_wrapper.type = 0;
  }
  
}

void ar_pose_reader::ar_pose_callback(const ar_pose::ARMarkers::ConstPtr& markers)
{
  //receives a list of marker poses and build a list of human_poses
  num_markers = markers->markers.size();
//   ROS_INFO("number of markers: [%d]", num_markers);
  
//   list_ped.humans.clear();

  for(int i=0; i< num_markers; i++){

	//feeding pose structure from ar_pose
	source_pose.header = markers->markers[i].header;
	source_pose.pose = markers->markers[i].pose.pose;
	marker_id = markers->markers[i].id;
    int confidence = markers->markers[i].confidence;
	
	//transform pose to /map frame
	try{
// 	  ros::Time now = ros::Time::now();
// 	  listener.waitForTransform("/prosilica_optical_frame", "/map", source_pose.header.stamp, ros::Duration(10.0));
// 	  listener.waitForTransform("/usb_cam", "/map", source_pose.header.stamp, ros::Duration(10.0));
	  listener.transformPose("/map",source_pose,target_pose);
	}
	catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

        
// 	ROS_INFO("confidence:%d",confidence);
        //only process markers with high confidence
        if(confidence > 60){
	
//           //retrieve pose values and feed into human_pose structure
//           local_ped.id = marker_id;
//           local_ped.header.frame_id = target_pose.header.frame_id;
//           local_ped.header.stamp = target_pose.header.stamp;
//           local_ped.x = target_pose.pose.position.x;
//           local_ped.y = target_pose.pose.position.y;
//           local_ped.theta = tf::getYaw(target_pose.pose.orientation); //in radians
//           local_ped.linear_velocity = 0.0;
//           local_ped.angular_velocity = 0.0;
//           list_ped.humans.push_back(local_ped); //append to the end of the list

          manage_list(marker_id);

          //check if there is a large jump in tracked markers
          double distance = euclidean_dist(
                            markers_list[marker_id].ghmm_wrapper.pose.x,
                            markers_list[marker_id].ghmm_wrapper.pose.y,
                            target_pose.pose.position.x,
                            target_pose.pose.position.y);
          
          if(markers_list[marker_id].ghmm_wrapper.type == 0 && distance > 5.0){
            ROS_INFO("distance:%f",distance);
            
//             ROS_INFO("not publishing");
          }
          
//           ROS_INFO("distance:%f",distance);


          //         if(markers_list[marker_id].type == 0 && distance > 5.0){
          //           ROS_INFO("distance:%f",distance);
          //           ROS_INFO("not publishing");
          //         } else{
          //ghmm wrapper
          markers_list[marker_id].ghmm_wrapper.object_id = marker_id;
//           markers_list[marker_id].ghmm_wrapper.type = markers_list[marker_id].type;
          markers_list[marker_id].ghmm_wrapper.header.frame_id = target_pose.header.frame_id;
          markers_list[marker_id].ghmm_wrapper.header.stamp = target_pose.header.stamp;
          markers_list[marker_id].ghmm_wrapper.pose.x = target_pose.pose.position.x;
          markers_list[marker_id].ghmm_wrapper.pose.y = target_pose.pose.position.y;
          markers_list[marker_id].ghmm_wrapper.pose.theta = tf::getYaw(target_pose.pose.orientation);//radians

          ROS_INFO("id:%d,type:%d",markers_list[marker_id].ghmm_wrapper.object_id,
                                   markers_list[marker_id].ghmm_wrapper.type);

          /// publish trajectories
          trajectory_pub.publish(markers_list[marker_id].ghmm_wrapper);
          //         }
        } else {
          ROS_INFO("confidence too low:%d",confidence);
        }
  }
  
}

void manage_list(int marker_id){
  markers_list[marker_id].detect_time = ros::Time::now();
  if(markers_list[marker_id].ghmm_wrapper.type == 2){
        markers_list[marker_id].ghmm_wrapper.type = 1; //first detection
        ROS_INFO("MARKER %d DETECTED",marker_id);
  }
  else if(markers_list[marker_id].ghmm_wrapper.type == 1){
        markers_list[marker_id].ghmm_wrapper.type = 0; //has been detected before
  }
}

// void manage_list(int marker_id){
//   markers_list[marker_id].detect_time = ros::Time::now();
//   if(markers_list[marker_id].type == 2){
// 	markers_list[marker_id].type = 1; //first detection
// 	ROS_INFO("MARKER %d DETECTED",marker_id);
//   }
//   else if(markers_list[marker_id].type == 1){
// 	markers_list[marker_id].type = 0; //has been detected before
//   }
// }

ar_humanProc::ar_humanProc()
{
  //advertise topic to publish human positions 
  pose_pub = n.advertise<social_filter::humanPoses>("human_poses", 15);

  //advertise topic to publish human positions for GHMM
  trajectory_pub = n.advertise<trajectory_simulator::TrajectoryObservation>("dynamic_objects", 15);
}

void ar_humanProc::init()
{
 	//instantiate reader class
	std_msgs::String msg;
	std::stringstream ss;
	ss<<"/ar_pose_marker";
	msg.data = ss.str();
// 	std_msgs::String ar_pose_topic = "/ar_pose_marker";
// 	ar_pose_reader* reader_ptr = new ar_pose_reader(ar_pose_topic, &n);
	ar_pose_reader* reader_ptr = new ar_pose_reader(msg, &n);
	reader_ptr->init();
	readers.push_back(reader_ptr);
}


void ar_humanProc::pub()
{
//     pose_pub.publish(list_ped);
}


ar_humanProc::~ar_humanProc()
{
//  for(unsigned int i=0; i< readers.size();i++)
//    delete readers[i];
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_human_processes");
  ar_humanProc h_processor;
  h_processor.init();
  ros::Rate loop_rate(10);

  while (ros::ok())
  {      
    h_processor.pub();
    ros::spinOnce();
    loop_rate.sleep();
	
        //iterate along n markers
	for(int i = 0; i < list_size; i++){
	  duration_since_detection = ros::Time::now() - markers_list[i].detect_time;
// 	  ROS_INFO("DURATION: %f", duration_since_detection.toSec());
          
          //if time in list entry is bigger then 
          //threshold and not type 2 change to type 2
	  if (duration_since_detection.toSec() > 1.0 && markers_list[i].ghmm_wrapper.type != 2){
// 		markers_list[i].type = 2;
		markers_list[i].ghmm_wrapper.type = 2;
		ROS_INFO("MARKER %d DISAPPEARED",i);
                
//                 markers_list[marker_id].ghmm_wrapper.object_id
                ROS_INFO("id:%d,type:%d",markers_list[i].ghmm_wrapper.object_id,
                                         markers_list[i].ghmm_wrapper.type);
                
		/// publish last maker position and type
		trajectory_pub.publish(markers_list[i].ghmm_wrapper);
	  }
	}
  }

  return 0;
}
 
