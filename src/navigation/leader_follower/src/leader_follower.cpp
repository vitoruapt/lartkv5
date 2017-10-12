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
#include "tf/tf.h"
#include "leader_follower.h"

#define DEG2RAD 0.01745



/*******
this node manages the choice of a leader and how the robot should follow it
********/

void target_pose_callback(const nav_msgs::Odometry::ConstPtr& msg){
  double distance_from_target = 2.0;
  double orientation = tf::getYaw(msg->pose.pose.orientation) + 1.57;
  double x_position = -msg->pose.pose.position.y - cos(orientation)*distance_from_target;
  double y_position = msg->pose.pose.position.x - sin(orientation)*distance_from_target;

  ROS_INFO("x pos:%f, y pos:%f",x_position,y_position);

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x_position;//goal.target_pose.pose.position.x + 0.1;//-msg->pose.pose.position.y;
  goal.target_pose.pose.position.y = y_position;//goal.target_pose.pose.position.y + 0.1;//msg->pose.pose.position.x;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);

}

void dyn_objects_callback(const trajectory_simulator::TrajectoryObservation::ConstPtr & dyn_objects){

  ///received a dynamic object, store as candidate
  // 	candidate_[dyn_objects->object_id] = *dyn_objects;
  // 	ROS_INFO("received dyn_objects id:%d",dyn_objects->object_id);
  ///check if received message refers to leader and if
  ///the robot should start following it, if so, track its path
  if(leader_id == dyn_objects->object_id && start_to_follow){
  if(leader_path.poses.size() > 0){
    //ROS_INFO("msg stamp:%d",dyn_objects->header.stamp.toSec());
    //ROS_INFO("last path stamp:%d",leader_path.poses.back().header.stamp.toSec());
    tlast = leader_path.poses.back().header.stamp.toSec();
    tnow = ros::Time::now().toSec();
    d = tnow - tlast;
    //ROS_INFO("elapsed time: %d",d);
    ///store path according to time step
    if(d > 2.0){
      leader_pose.pose.position.x = dyn_objects->pose.x;
      leader_pose.pose.position.y = dyn_objects->pose.y;
      leader_pose.pose.orientation = tf::createQuaternionMsgFromYaw(dyn_objects->pose.theta + 1.57);
      leader_pose.header.stamp = ros::Time::now();
      leader_path.poses.push_back(leader_pose);
    }
  }
  else{
    ///only executes on first message
    path_initialized = true;
    leader_pose.pose.position.x = dyn_objects->pose.x;
    leader_pose.pose.position.y = dyn_objects->pose.y;
    leader_pose.pose.orientation = tf::createQuaternionMsgFromYaw(dyn_objects->pose.theta + 1.57);
    leader_pose.header.stamp = ros::Time::now();
    leader_path.poses.push_back(leader_pose);
  }

  ///simple following part  
  // 	  try{
  // 		listener->waitForTransform("/robot_0/base_link", "/Hiro", ros::Time(0), ros::Duration(10.0));
  // 		listener->lookupTransform("/robot_0/base_link", "/Hiro", ros::Time(0), transform);
  // // 		listener->lookupTransform("/robot_0/base_link", "/Hiro", ros::Time::now()/*(0)*/, transform);
  // // 		no_track_error=false;
  // 	  }
  // 	  catch (tf::TransformException ex){
  // // 		cmd_vel.angular.z = 0; //if the kinect loose traking of the person, stop the wheelchair.
  // // 		cmd_vel.linear.x = 0;
  // // 		no_track_error=true;
  // 		ROS_ERROR("%s",ex.what());
  // 	  }

  // 	  ROS_INFO("robotx:%f,roboty:%f,objx:%f,objy:%f",
  // 		robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y,
  // 		dyn_objects->pose.x, dyn_objects->pose.y);
  // 		
  // 	  dist = euclidean_dist(
  // 		robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y,
  // 		dyn_objects->pose.x, dyn_objects->pose.y);
  // 		
  // 	  theta = atan2(
  // 		robot_pose.pose.pose.position.x - dyn_objects->pose.x,
  // 		robot_pose.pose.pose.position.y - dyn_objects->pose.y);
  // 	  
  // 	  //controller part
  // 	  dist = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));	
  // 	  theta = atan2(transform.getOrigin().y(),transform.getOrigin().x());
  // 	  
  // 	  ROS_INFO("distance:%f, theta:%f",dist,theta);
  // 	  if (dist>=dmin && dist<=dmax && theta>=-theta_max && theta<=theta_max)
  // 	  {
  // 		  cmd_vel.angular.z = 1.6 * (theta/theta_max) ;
  // 		  cmd_vel.linear.x = 1.2 * (dist-dmin)/(dmax-dmin);
  // 		  ROS_INFO("cmd ang:%f, cmd lin:%f",cmd_vel.angular.z,cmd_vel.linear.x);
  // 		  robot_cmd_vel.publish(cmd_vel);
  // 	  }
  // 	  else 
  // 	  {
  // 		  cmd_vel.angular.z = 0;
  // 		  cmd_vel.linear.x = 0;
  // 	  }
  }
}

// void ar_pose_callback(const ar_pose::ARMarkers::ConstPtr& markers){
// 	//receives a list of marker and chooses a leader and send pose as goal
// 	//   num_markers = markers->markers.size();
// 	
// 	num_markers = 1;
// 	//   ROS_INFO("number of markers: [%d]", num_markers);
// 
// 	for(int i=0; i< num_markers; i++){
// 
// 	  //feeding pose structure from ar_pose
// 	  source_pose.header = markers->markers[i].header;
// 	  source_pose.pose = markers->markers[i].pose.pose;
// 	  marker_id = markers->markers[i].id;
// 	  
// 	  ROS_INFO("before listener");
// 	  //transform pose to /map frame
// 	  try{
// 		ros::Time now = ros::Time::now();
// 		listener->waitForTransform("/prosilica_optical_frame", "/map", source_pose.header.stamp, ros::Duration(10.0));
//   // 	  listener.waitForTransform("/usb_cam", "/map", source_pose.header.stamp, ros::Duration(10.0));
// 		listener->transformPose("/map",source_pose,target_pose);
// 	  }
// 	  catch (tf::TransformException ex) {
// 		ROS_ERROR("%s", ex.what());
// 		return;
// 	  }
// 
// 	  goal.target_pose.header.frame_id = "/map";
// 	  goal.target_pose.header.stamp = ros::Time::now();
// 	  goal.target_pose.pose = target_pose.pose;
// 	}
// }

void leader_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
// 	ROS_INFO("got leader goal pose");
// 	leader_goal = *msg;
  int temp_id = msg->pose.position.z;
  leader_goal[temp_id].pose = msg->pose;
  leader_goal[temp_id].header = msg->header;
}

void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg){
// 	ROS_INFO("got robot goal pose");
// 	robot_goal = *msg;
  robot_goal.pose = msg->pose;
  robot_goal.header = msg->header;
}

void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg){
// 	ROS_INFO("got robot pose");
// 	robot_pose = *msg;
  robot_pose.pose = msg->pose;
  robot_pose.header = msg->header;
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "leader_follower");
  ros::NodeHandle n;

  //used in transformations
  tf::TransformListener _listener;
  listener = &_listener;
  
  //initialize leader_path
  leader_path.header.stamp = ros::Time::now();
  leader_path.header.frame_id = "/map";
  
  //tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("robot_0/move_base", true);
  
  //subscribers
  //   pose_subscriber = n.subscribe("/robot_1/base_pose_ground_truth", 15, &target_pose_callback);
  //   ar_pose_sub = n.subscribe("/ar_pose_marker", 15, &ar_pose_callback);  
  dyn_objects_subscriber = n.subscribe("dynamic_objects", 100, &dyn_objects_callback);
  leader_goal_subscriber = n.subscribe("leader_goal_pose",10,&leader_goal_callback);
  robot_goal_subscriber = n.subscribe("/robot_0/goal", 1, &robot_goal_callback);
  robot_pose_subscriber = n.subscribe("/robot_0/amcl_pose", 1, &robot_pose_callback);
  
  //publishers
  pathPublisher = n.advertise<nav_msgs::Path>("leader_path", 1);
  robot_cmd_vel = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1);
  next_posePublisher = n.advertise<geometry_msgs::PoseStamped>("/robot_0/leader_next_pose", 1);

//wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }
  
  //initialize struct
  for(int i = 0; i < 4; i++){//number of markers
    leader_goal[i].pose.position.x = 99;
    leader_goal[i].pose.position.y = 99;
  }
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()){
	
    ///compute distance from ROBOT to its goal
    robot2goal_distance = euclidean_dist(
    robot_pose.pose.pose.position.x,robot_pose.pose.pose.position.y,
    robot_goal.pose.position.x,robot_goal.pose.position.y);
	  
    ///compute distance from LEADER to its goal (after leader has been found)
    if(leader_found){
      leader2goal_distance = euclidean_dist(
      leader_pose.pose.position.x,leader_pose.pose.position.y,
      leader_goal[leader_id].pose.position.x,leader_goal[leader_id].pose.position.y);
    }
// 	goals_distance = euclidean_dist(
// 	  candidate_[i].pose.position.x,candidate_[i].pose.position.y,
// 	  robot_goal.pose.position.x,robot_goal.pose.position.y);
	  
// 	ROS_INFO("goal distances, robot:%.1f, leader:%f.1",robot2goal_distance,leader2goal_distance);
    ///leader selection criteria, distance between goals
    for(int i = 0; i < 4; i++){//number of markers
      goals_distance = euclidean_dist(
            leader_goal[i].pose.position.x,leader_goal[i].pose.position.y, 
            robot_goal.pose.position.x,robot_goal.pose.position.y);
      ROS_INFO("candidate id:%d, goals:%f",i,goals_distance);
      
      ///artificial leader selection
      //if(i == 0) goals_distance = 2;
      
      if(goals_distance < 3.0){
// 		//candidate is a leader
        leader_found = true;
        if(leader_id != i){ ///check if it is a new leader
          leader_id = i;
          ROS_INFO("found new leader, id:%d",leader_id);
          //using z coordinate to obtain id
          path_initialized = false;
          start_to_follow = false;
          //initialize path structure
          leader_path.header.stamp = ros::Time::now();
          leader_path.header.frame_id = "/map";
          leader_path.poses.clear();
        }
        //check if leader is closer to the goal than the robot is
// 		ROS_INFO("leader2goal dist: %f, robot2goal dist: %f", leader2goal_distance, robot2goal_distance);
        if(/*leader2goal_distance > 1.0 && leader2goal_distance < 1.0*robot2goal_distance*/true){
          ROS_INFO("leader %d is closer, start to follow",leader_id);
          start_to_follow = true;
        }
      break; //can skip candidate search as leader has been found
      }
    }
	 
	//leader decision part
// 	if(goals_distance < 5.0){
// 	if(leader_found){
// 	  //leader found
// 	  if(leader_id != leader_goal.pose.position.z){ ///check if it is a new leader
// 		ROS_INFO("found leader, id:%d",leader_id);
// 		//using z coordinate to obtain id
// 		leader_id = leader_goal.pose.position.z;
// 		path_initialized = false;
// 		start_to_follow = false;
// 		//initialize path structure
// 		leader_path.header.stamp = ros::Time::now();
// 		leader_path.header.frame_id = "/map";
// 		leader_path.poses.clear();
// 	  }
// 	  //check if leader is closer to the goal than the robot is
// 	  ROS_INFO("leader2goal dist: %f, robot2goal dist: %f", leader2goal_distance, robot2goal_distance);
// 	  if(leader2goal_distance > 1.0 && leader2goal_distance < 1.0*robot2goal_distance){
// 		ROS_INFO("leader is closer, start to follow");
// 		start_to_follow = true;
// 	  }
// 	}
	
	if(path_initialized){///can sending goals
	  if(send_goal){
            ///set subgoal as first path position as goal
            goal.target_pose.header.frame_id = "/map";
            goal.target_pose.header.stamp = ros::Time::now();	
            goal.target_pose.pose.position.x = leader_path.poses[0].pose.position.x;
            goal.target_pose.pose.position.y = leader_path.poses[0].pose.position.y;
            goal.target_pose.pose.orientation = leader_path.poses[0].pose.orientation;
            
            ///publish subgoal to RiskRRT
            next_pose.header.frame_id = "/map";
            next_pose.header.stamp = ros::Time::now();
            next_pose.pose = goal.target_pose.pose;
            next_posePublisher.publish(next_pose);
            
            ROS_INFO("Sending goal");
  // 		ac.sendGoal(goal);
            send_goal = false;
	  }

// 	  ROS_INFO("robotx:%f,roboty:%f,goalx:%f,goaly:%f",
// 		robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y,
// 		leader_path.poses[0].pose.position.x, leader_path.poses[0].pose.position.y);

	  ///distance from robot to subgoal
	  double distance_to_pose = euclidean_dist(
            robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y,
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
// 	  ROS_INFO("ROBOT DISTANCE TO GOAL: %f", distance_to_pose);

	  ///check subgoal success and erase first list element of path
// 	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	  if(distance_to_pose < 2.0){
            if(leader_path.poses.size() > 1){
              leader_path.poses.erase(leader_path.poses.begin());
              send_goal = true;
            }
	  }
	}

	pathPublisher.publish(leader_path);
	ros::spinOnce();
	loop_rate.sleep();


  }
  return 0;
}
 
