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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
/**
 * @file test.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief small test
 */
#include <unistd.h>
#include <iostream>
#include <math.h>

#define PI 3.141592653589793238462643383279502884197


using namespace ros;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_test");
    
    tf::TransformBroadcaster br_tf;
    tf::Transform tf_right_feet;
    tf::Transform tf_left_feet;
    ros::Rate loop_rate(1000);
    
    tf_left_feet.setOrigin( tf::Vector3(0.0, 0.07, 0.0) );
    tf_left_feet.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, 0.0) );
    tf_right_feet.setOrigin( tf::Vector3(0.0, -0.07, 0.0) );
    tf_right_feet.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, 0.0) );

    //double valz=0, valy=0, d=0;
    
    while (ros::ok())
    {
        //valz=0.2 + 0.2*cos(d);
        //valy=0.0 + 0.35*sin(d);
        //tf_left_feet.setOrigin( tf::Vector3(valy, 0.10, valz) );
        //d+=0.001;
        
        
        br_tf.sendTransform(tf::StampedTransform(tf_left_feet, ros::Time::now(), "/world", "/tf_left_feet"));
        
        br_tf.sendTransform(tf::StampedTransform(tf_right_feet, ros::Time::now(), "/world", "/tf_right_feet"));
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
