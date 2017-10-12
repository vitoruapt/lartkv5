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
#ifndef _atlascar_transforms_CPP_
#define _atlascar_transforms_CPP_

/**
 * @file  
 * @brief A deprectated code that published the atlascar transforms. This is now
 * done by the robot state publisher.

 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */

#include "state_example.h"

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "atlascar_transforms"); // Initialize ROS coms
	ros::NodeHandle n; //The node handle
	p_node = &n;
	ros::Rate loop_rate(1);	

	tf::Vector3 V(-1.8583, 0, 0.9927);
	tf::Matrix3x3 M(0.9744, 0, 0.2250, 0, 1, 0, -0.225, 0, 0.9744);
	double r,p,y;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0
	M.getEulerYPR(y, p, r); //Changed by V. Santos, 15-Abr-2013,09:11
#else // Code for earlier versions (Electric, diamondback, ...)
	M.getRPY(r,p,y);
#endif
	ROS_INFO("Link position (to copy paste directly to urdf)");
	ROS_INFO("xyz=\"%f %f %f\" rpy=\"%f %f %f\"", V[0], V[1], V[2], r,p,y);

	//TRANSFORM FROM LASER_LEFT_BUMPER to Vehicle (A transform that brings a point in)
	tf::TransformBroadcaster tf_br1; 
	tf::Transform tr1 = tf::Transform(tf::Matrix3x3(0.8039, 0.5948, 0, -0.5948, 0.8039, 0, 0, 0, 1),
			tf::Vector3(-0.08, 0.8350, 0));

	//TRANSFORM FROM LASER_RIGHT_BUMPER to Vehicle (A transform that brings a point in)
	tf::TransformBroadcaster tf_br2; 
	tf::Transform tr2 = tf::Transform(tf::Matrix3x3(-0.8192,0.5736,0,-0.5736,-0.8192,0,0,0,1),
			tf::Vector3(-0.12, -0.89, -0.0950));

	//TRANSFORM FROM LASER_ROOF_ROTATING_BASE to Vehicle (A transform that brings a point in)
	tf::TransformBroadcaster tf_br3; 
	tf::Transform tr3 = tf::Transform(tf::Matrix3x3(0.9890,0.0129,0.1472,0.,0.9962,-0.0872,-0.1478,0.0862,0.9853),
			tf::Vector3(-1.7803, 0, 1.1861));

	//TRANSFORM FROM XB3_right camera to Vehicle (A transform that brings a point in)
// 	tf::TransformBroadcaster tf_br4; 
// 	tf::Transform tr4 = tf::Transform(tf::Matrix3x3(0,-0.1392, 0.9903 , -1, 0, 0, 0, -0.9903, -0.1392),
// 			tf::Vector3(-1.8405, 0.2400, 1.1170));

	//TRANSFORM FROM PTU base to Vehicle (A transform that brings a point in)
// 	tf::TransformBroadcaster tf_br5; 
// 	tf::Transform tr5 = tf::Transform(tf::Matrix3x3(1,0,0,0,1,0,0,0,1),
// 			tf::Vector3(-1.877, -0.4920, 0.99));

	//TRANSFORM FROM Hokuyo laser to Vehicle (A transform that brings a point in)
// 	tf::TransformBroadcaster tf_br6; 
// 	tf::Transform tr6 = tf::Transform(tf::Matrix3x3(0.9744, 0, 0.2250, 0, 1, 0, -0.225, 0, 0.9744),
// 			tf::Vector3(-1.8583, 0, 0.9927));

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	tf::TransformBroadcaster broadcaster;

	const double degree = M_PI/180;

	// robot state
	double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "axis";

	while (ros::ok()) {
		//update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(3);
		joint_state.position.resize(3);
		joint_state.name[0] ="swivel";
		joint_state.position[0] = swivel;
		joint_state.name[1] ="tilt";
		joint_state.position[1] = tilt;
		joint_state.name[2] ="periscope";
		joint_state.position[2] = height;


		// update transform
		// (moving in a circle with radius=2)
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = cos(angle)*2;
		odom_trans.transform.translation.y = sin(angle)*2;
		odom_trans.transform.translation.z = .7;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

		//send the joint state and transform
		joint_pub.publish(joint_state);
		broadcaster.sendTransform(odom_trans);

		// Create new robot state
		tilt += tinc;
		if (tilt<-.5 || tilt>0) tinc *= -1;
		height += hinc;
		if (height>.2 || height<0) hinc *= -1;
		swivel += degree;
		angle += degree/4;

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}


	//JUST ADD the sensors here, their 6dof position, and add a transform broadcaster at the while cycle

	while (n.ok()) //CYCLE, because transforms are static, will publish a transform once a second
	{

		tf_br1.sendTransform(tf::StampedTransform(tr1, ros::Time::now(), "/tf_atc/vehicle","/tf_atc/laser/left_bumper"));
		tf_br2.sendTransform(tf::StampedTransform(tr2, ros::Time::now(), "/tf_atc/vehicle","/tf_atc/laser/right_bumper"));
		tf_br3.sendTransform(tf::StampedTransform(tr3, ros::Time::now(), "/tf_atc/vehicle","/tf_atc/laser/roof_rotating_base"));
		//tf_br4.sendTransform(tf::StampedTransform(tr4, ros::Time::now(), "/tf_atc/laser/roof_rotating_base","/tf_atc/laser/roof_rotating"));

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
#endif
