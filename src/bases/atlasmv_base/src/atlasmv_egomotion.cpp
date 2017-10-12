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
 * \brief Obtain the vehicle egomotion from internal sensors
 */

#include <iostream>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define DEFAULT_WHEEL_BASE 0.497

#include <signal.h>

#include <atlasmv_base/AtlasmvStatus.h>

using namespace ros;
using namespace std;

void IncommingDataHandler(int);
bool ConvertEstimatedToMeasurment(double vl,double dir,double*dx,double*dy,double*dtheta,double dt,double l,double bwa);

bool new_status_message;
atlasmv_base::AtlasmvStatus base_status;

void StatusMessageHandler(const atlasmv_base::AtlasmvStatus& msg)
{
	base_status=msg;
	new_status_message=true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "atlasmv_egomotion");
	
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/atlasmv/base/odometry", 50);     // atmv??
	tf::TransformBroadcaster odom_broadcaster;
	
	Subscriber subscribeStatusMessages = n.subscribe ("/atlasmv/base/status", 1, StatusMessageHandler);     // atmv??
	
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	
	double vl,phi;
	double l;
	
	n.param("wheel_base",l,DEFAULT_WHEEL_BASE);
	
	new_status_message=false;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(60.0);
	
	ROS_INFO("Starting to spin ...");
	
	while(n.ok())
	{
		spinOnce();	
		r.sleep();
		
		if(!new_status_message)
			continue;
		
		new_status_message=false;
		
		current_time = ros::Time::now();
		
		vl=base_status.speed;
		phi=base_status.dir;
		
		//compute odometry
		double dt = (current_time - last_time).toSec();
		
		double delta_x;
		double delta_y;
		double delta_th;
				
		delta_x=cos(th)*cos(phi)*vl*dt;
		delta_y=sin(th)*cos(phi)*vl*dt;
		delta_th=sin(phi/l)*vl*dt;
                
                ROS_INFO("cos(th):%f,cos(phi):%f",cos(th),cos(phi));
                ROS_INFO("dx:%f,dy:%f,dth:%f",delta_x,delta_y,delta_th);
                ROS_INFO("th:%f,phi:%f,vl:%f",th,phi,vl);
                
                vx = delta_x/dt;
                vy = delta_y/dt;
                vth = delta_th/dt;
		
		x += delta_x;
		y += delta_y;
		th += delta_th;
                
                ROS_INFO("dt:%f",dt);
                ROS_INFO("x:%f,y:%f,th:%f",x,y,th);
                ROS_INFO("vx:%f,vy:%f,vth:%f",vx,vy,vth);
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/odom";
		odom_trans.child_frame_id = "/base_link";
		
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "/odom";
		
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		
		
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "/base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
		
		//publish the message
		odom_pub.publish(odom);
		
		last_time = current_time;
	}
}
