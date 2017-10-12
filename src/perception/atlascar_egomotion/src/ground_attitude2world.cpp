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
 * \file
 * \brief Ground attitude to world transform publication
 */

#include<tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <stdio.h>
#include <tf/transform_broadcaster.h>
ros::Publisher odom_pub;
nav_msgs::Odometry odometry_;
using namespace std;

#define PFLN {printf("%s %d\n",__FILE__, __LINE__);}
tf::TransformBroadcaster *p_odom_broadcaster;
sensor_msgs::Imu imu_data;
//tf::TransformBroadcaster *p_tf_broadcaster;
tf::TransformListener *p_listener;

//FILE * pFile;

//this 
void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg )
{
	imu_data=*msg;
}
int main(int argc, char** argv)
{ 
//	pFile=fopen("../alboi_final_pos.txt","w+");
	ros::init(argc, argv, "robot_pose2odometry");
	ros::NodeHandle n;
	odom_pub = n.advertise<nav_msgs::Odometry>("/robot_pose_odometry", 50);
	ros::Subscriber imu= n.subscribe("/atc/imu/data",1, ImuCallback);
	tf::TransformListener listener;
	//p_listener=&listener;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Rate r(40.0);
    while(n.ok())
       {   
              ros::spinOnce();   
              r.sleep();	
              //intersect transformation by robot_pose_ekf
              bool have_transform=false;
              tf::StampedTransform transform_robot_pose_ekf;
              try
              {
                     listener.lookupTransform("/world","/base_footprint" , ros::Time(0), transform_robot_pose_ekf);
                     have_transform=true;
              }
              catch (tf::TransformException ex)
              {
                     if(listener.waitForTransform("/world","/base_footprint",ros::Time(0), ros::Duration(3.0)))
                     {
                            try
                            {
                                   listener.lookupTransform("/world","/base_footprint" , ros::Time(0), transform_robot_pose_ekf);
                                   have_transform=true;
                            }
                            catch (tf::TransformException ex)
                            {
                                   ROS_ERROR("Could not lookup transform after waiting 3 secs\n.%s",ex.what());
                            }
                     }
                     else
                     {
                            ROS_ERROR("Could find valid transform after waiting 3 secs\n.%s",ex.what());
                     }
              }

              if (have_transform==false)
              {
              //return;
                     continue;
              }
/*
              //intersect transformation from center_car_axes to /road_plane
              bool have_transform_2=false;
              tf::StampedTransform transform2ground;
              try
              {
                     listener.lookupTransform("/ground","/atc/vehicle/center_car_axes" , ros::Time(0), transform2ground);
                     have_transform_2=true;
              }
              catch (tf::TransformException ex)
              {
                     if(listener.waitForTransform("/ground","/atc/vehicle/center_car_axes",ros::Time(0), ros::Duration(3.0)))
                     {
                            try
                            {
                                   listener.lookupTransform("/ground","/atc/vehicle/center_car_axes" , ros::Time(0), transform2ground);
                                   have_transform_2=true;
                            }
                            catch (tf::TransformException ex)
                            {
                                   ROS_ERROR("Could not lookup transform after waiting 3 secs\n.%s",ex.what());
                            }
                     }
                     else
                     {
                            ROS_ERROR("Could find valid transform after waiting 3 secs\n.%s",ex.what());
                     }
              }

              if (have_transform_2==false)
              {
              //return;
                     continue;
              }

              //get roll pitch yaw from transform2ground
              double roll_2,pitch_2,yaw_2;
              btQuaternion q_2 = transform2ground.getRotation();
              btMatrix3x3(q_2).getRPY(roll_2, pitch_2, yaw_2);
*/

              //get roll pitch yaw from transform_robot_pose_ekf
              double roll_1,pitch_1,yaw_1;
              tf::Quaternion q1 = transform_robot_pose_ekf.getRotation();
              tf::Matrix3x3(q1).getRPY(roll_1, pitch_1, yaw_1);
              static tf::TransformBroadcaster br;
              tf::Transform transform;
              transform.setOrigin( tf::Vector3((transform_robot_pose_ekf.getOrigin()).x(), (transform_robot_pose_ekf.getOrigin()).y(), 0) );
              transform.setRotation(tf::createQuaternionFromRPY(0.0,0.0, yaw_1) );
              //transform.setRotation(tf::createQuaternionFromRPY(0.0,0.0, yaw_1) );
              br.sendTransform(tf::StampedTransform(transform, transform_robot_pose_ekf.stamp_,"/world", "/ground" ));
	
              //        fprintf(pFile,"%0.5f %0.5f\n", (transform_robot_pose_ekf.getOrigin()).x(), (transform_robot_pose_ekf.getOrigin()).y());
       }
//	fclose(pFile);
	return 0;
}
