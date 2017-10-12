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
 * \brief File that adds the IMU odometry covariance to the estimation
 */

#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
// #include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
using namespace std;

#include <iostream>
#include <fstream>

nav_msgs::Odometry odometry_;
sensor_msgs::Imu inercial_;
ros::Time t_i;
ros::Time t_i_1;
ros::Publisher odom_pub;
ros::Publisher imu_pub;

tf::TransformBroadcaster *p_odom_broadcaster;
/**
 * \brief ImuCallback Function to add covariance to IMU data 
*/
void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg )
{
      // ROS_INFO("imu message received");
   //    inercial_=*msg;
       inercial_.header.stamp=msg->header.stamp;//mudar!!


       tf::StampedTransform transform;

       //Transformation necessary to the robot_pose_ekf node
       transform.stamp_=msg->header.stamp;
       transform.frame_id_="base_footprint";
       transform.child_frame_id_ = "/imu_frame";	
       tf::Vector3 transl(0,0,0);
       transl[0]=0; 
       transl[1]=0; 
       transl[2]=0;
       transform.setOrigin(transl);

       tf::Quaternion odom_quat_ = tf::createQuaternionFromRPY(0,0,0);
       transform.setRotation(odom_quat_);
       //Publicar nova tf
       p_odom_broadcaster->sendTransform(transform);


       inercial_.header.frame_id = "/imu_frame";
              
       tf::Quaternion orientation;
       tf::quaternionMsgToTF(msg->orientation, orientation);
       double roll,pitch,yaw;
       tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
       //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(yaw,-pitch,-roll);
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
      
       inercial_.orientation=odom_quat;
       inercial_.orientation_covariance[0]= 0.000001;
       inercial_.orientation_covariance[4]= 0.000001;
       inercial_.orientation_covariance[8]= 0.000001;
       inercial_.angular_velocity_covariance[0]= 0.000304617;
       inercial_.angular_velocity_covariance[4]= 0.000304617;
       inercial_.angular_velocity_covariance[8]= 0.000304617;
       inercial_.linear_acceleration_covariance[0]= 0.0000004;
       inercial_.linear_acceleration_covariance[4]= 0.0000004;
       inercial_.linear_acceleration_covariance[8]= 0.0000004;
       imu_pub.publish(inercial_);

}

void OdometryCallback (const nav_msgs::Odometry::ConstPtr &msg )
{
      // ROS_INFO("odometry message received");
       odometry_=*msg;
       odometry_.header.stamp=msg->header.stamp;
       odometry_.header.frame_id="/world";
       odometry_.child_frame_id="base_footprint";
       
      odometry_.pose.covariance[0]=msg->pose.covariance[0];
      odometry_.pose.covariance[7]=msg->pose.covariance[7];
      odometry_.pose.covariance[14]=99999999999;
      odometry_.pose.covariance[21]=99999999999;
      odometry_.pose.covariance[28]=99999999999;
      odometry_.pose.covariance[35]=msg->pose.covariance[35];
       
      
      odometry_.twist.covariance[0]=999999;
      odometry_.twist.covariance[7]=999999;
      odometry_.twist.covariance[14]=999999;
      odometry_.twist.covariance[21]=999999;
      odometry_.twist.covariance[28]=999999;
      odometry_.twist.covariance[35]=999999;
       
      odom_pub.publish(odometry_);
              

}



//this node just completes the information necessary to robot_pose_ekf
int main(int argc, char** argv)
{


       ros::init(argc, argv, "imu_egomotion");
       ROS_INFO("Starting message edition node");
       ros::NodeHandle n;

	tf::TransformBroadcaster odom_broadcaster;
	p_odom_broadcaster=&odom_broadcaster;
       
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 50);

       ros::Subscriber imu= n.subscribe("/atc/imu/data",1, ImuCallback);
       ros::Subscriber odom= n.subscribe("/vhc/odometry",1, OdometryCallback);

        
        ros::Rate loop_rate(1000);
        ros::spin();

       return 0;
}
