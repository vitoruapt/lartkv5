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
 * @file imu_rviz.cpp
 * @author Telmo Rafeiro n.º 45349 (rafeiro@ua.pt)
 * @brief Toll for IMU's data visualization - only 8 IMU
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <math.h> 
#include <cmath>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/format.hpp> 

#include <imu_network/filtered_imu.h>
#include <imu_network/filtered_imu_network.h>

#include <kfilter/ekfilter.hpp>
#include <kfilter/kvector.hpp>

// #include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <cfloat>
#include <float.h>
#include <assert.h>


#define MAX_MEAN_IT 64 //number of iterations to measure the mean values from gyroscopes

#define GYRO_CONVERT M_PI/180*0.069565 // conversion of gyro's values from ADC to rad/sec
#define DEGtoRAD M_PI/180

using namespace std;

ros::NodeHandle* n;

tf::TransformBroadcaster* br;
tf::TransformListener* listener;

ros::Publisher chatter_pub;
ros::Publisher vis_pub;


void publish_accel(int n, float ax , float ay , float az);
void publish_gyro(int n, float gx , float gy , float gz);
void publish_magn(int n, float mx , float my , float mz);
void publish_sensor_type(void);
void publish_sensor_number(int n);

float get_theta(float iteration1, float iteration2,float time_interval);
float get_magnitude(float x, float y, float z);
float get_angle(float num, float denom);


int sensors_number;
int first_run=0;
int marker_id;
int mean_it = 0;
tf::Transform transform1;
tf::Transform transform_tmp;

vector<ros::Time>time_stamp;
vector<float>gyro_valx;
vector<float>gyro_valy;
vector<float>gyro_valz;

vector<float>gyro_mean_vect[MAX_MEAN_IT][3]; //vectors for allocation of values of gyro for mean
vector<float>gyro_real_mean[3]; //vetor of gyro's means

int sensors_pos_y[9] = {0 , 5 , 30 , 35 , 8.75 , 8.75 , 26.25 , 26.25 , 17.5};
int sensors_pos_z[9] = {0 , 5 , 5 , 0 , -20 , -35 , -20 , -35 , 15 };
tf::Matrix3x3 imu_transform[9];
tf::Matrix3x3 imu_transform_result;

void chatterCallback(const imu_network::filtered_imu_network::ConstPtr& msg)
{
    double RPY_orient[3];
    double RPY_init[3];
    if(first_run ==0)
    {
        imu_transform[0].setValue(1,0,0,0,0,1,0,-1,0);
		imu_transform[1].setValue(1,0,0,0,1,0,0,0,1);
        imu_transform[2].setValue(1,0,0,0,1,0,0,0,1);
        imu_transform[3].setValue(1,0,0,0,0,1,0,-1,0);
        imu_transform[4].setValue(0,1,0,0,0,1,1,0,0);
        imu_transform[5].setValue(0,1,0,0,0,1,1,0,0);
        imu_transform[6].setValue(0,-1,0,0,0,-1,1,0,0);
        imu_transform[7].setValue(0,-1,0,0,0,-1,1,0,0);
        imu_transform[8].setValue(-1,0,0,0,0,-1,0,-1,0);
        sensors_number=(int)msg->filtered_imu_network[0].total_number;
        
        for (int i=0;i<sensors_number;i++)
        {

	  // tf reference systems _ definition
	  string sensor_base = str( boost::format("/sensor_base_%d") % i );
	  string sensor  = str( boost::format("/sensor_%d") % i );
	  // setting initial values of reference systems
	  transform1.setOrigin( tf::Vector3(0.0, sensors_pos_y[i], sensors_pos_z[i]) );
	  transform1.setRotation( tf::createQuaternionFromRPY(0,0,0) );
	  br->sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/world", sensor_base));
	  
	  imu_transform[i].getRPY(RPY_init[0],RPY_init[1],RPY_init[2]);
	  
	  transform_tmp.setOrigin( tf::Vector3(0.0,0.0,0.0) );
	  transform_tmp.setRotation(tf::createQuaternionFromRPY(RPY_orient[0],RPY_orient[1],RPY_orient[2]) );
	  br->sendTransform(tf::StampedTransform(transform_tmp, ros::Time::now()-ros::Duration(5),sensor_base, sensor));
	  
	  gyro_valx.push_back(msg->filtered_imu_network[i].angular_velocity_x);
	  gyro_valy.push_back(msg->filtered_imu_network[i].angular_velocity_y);
	  gyro_valz.push_back(msg->filtered_imu_network[i].angular_velocity_z);

	  cout<<gyro_valx[i]<<" "<<gyro_valy[i]<<" "<<gyro_valz[i]<<endl;

        }
        first_run = 1 ;  
        return;
    }

    float sensor_var[sensors_number][13];
        
    marker_id = 0;
    
    for(int i = 0;i<sensors_number;i++)
    {
        //getting the sensors_values from /topic_raw_data
        sensor_var[i][0]=msg->filtered_imu_network[i].linear_acceleration_x;
        sensor_var[i][1]=msg->filtered_imu_network[i].linear_acceleration_y;
        sensor_var[i][2]=msg->filtered_imu_network[i].linear_acceleration_z;
        sensor_var[i][3]=msg->filtered_imu_network[i].angular_velocity_x;
        sensor_var[i][4]=msg->filtered_imu_network[i].angular_velocity_y;
        sensor_var[i][5]=msg->filtered_imu_network[i].angular_velocity_z;
        sensor_var[i][6]=msg->filtered_imu_network[i].angular_displacement_x;
        sensor_var[i][7]=msg->filtered_imu_network[i].angular_displacement_y;
        sensor_var[i][8]=msg->filtered_imu_network[i].angular_displacement_z;
        sensor_var[i][9]=msg->filtered_imu_network[i].magnetometer_x;
        sensor_var[i][10]=msg->filtered_imu_network[i].magnetometer_y;
        sensor_var[i][11]=msg->filtered_imu_network[i].magnetometer_z;
        sensor_var[i][12]=msg->filtered_imu_network[i].number;
        

     
        string sensor_base = str( boost::format("/sensor_base_%d") % i );
        string sensor  = str( boost::format("/sensor_%d") % i );
        
        
        transform1.setOrigin( tf::Vector3(0.0, sensors_pos_y[i], sensors_pos_z[i]) );
        transform1.setRotation( tf::createQuaternionFromRPY(0,0,0) );
        br->sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/world", sensor_base));

        
        tf::StampedTransform transform_new;
        tf::StampedTransform transform_sensors;
        try
        {
	  float delta_thetax = 0;
	  float delta_thetay = 0;
	  float delta_thetaz = 0;
	  
	  listener->lookupTransform(sensor_base, sensor, ros::Time(0), transform_new);
	  tf::Quaternion q;
	  q=transform_new.getRotation();
	  tf::Matrix3x3(q).getRPY(RPY_orient[0],RPY_orient[1],RPY_orient[2]);
	  

	  delta_thetax = get_theta(gyro_valx[i],sensor_var[i][3],0.025);
	  delta_thetay = get_theta(gyro_valy[i],sensor_var[i][4],0.025);
	  delta_thetaz = get_theta(gyro_valz[i],sensor_var[i][5],0.025);

	  if (msg->filtered_imu_network[i].algorithm == 0)
	  {
	      ROS_INFO("KALMAN_FILTER");
	      transform_tmp.setRotation( tf::createQuaternionFromRPY(RPY_orient[0]+delta_thetax,RPY_orient[1]+delta_thetay,RPY_orient[2]+delta_thetaz)/* * tf::createQuaternionFromRPY(RPY_init[0],RPY_init[1],RPY_init[2])*/);
	      transform_tmp.setOrigin(tf::Vector3(0,0,0));
	      br->sendTransform(tf::StampedTransform(transform_tmp, ros::Time::now(), sensor_base, sensor));

	      gyro_valx[i] = sensor_var[i][3];
	      gyro_valy[i] = sensor_var[i][4];
	      gyro_valz[i] = sensor_var[i][5];
	  }
	  if (msg->filtered_imu_network[i].algorithm == 1)
	  {
	      ROS_INFO("COMPLEMENTARY_FILTER");
	      
	      tf::Quaternion quat = tf::createQuaternionFromRPY(sensor_var[i][6],sensor_var[i][7],sensor_var[i][8]);
	      imu_transform_result = tf::Matrix3x3(quat) * imu_transform[i].inverse();
	      imu_transform_result.getRPY(RPY_init[0],RPY_init[1],RPY_init[2]);
	      transform_tmp.setRotation(tf::createQuaternionFromRPY(RPY_init[0],RPY_init[1],RPY_init[2]));
	  
		  transform_tmp.setRotation(tf::createQuaternionFromRPY(sensor_var[i][6],sensor_var[i][7],sensor_var[i][8]) * tf::createQuaternionFromRPY(RPY_init[0],RPY_init[1],RPY_init[2]).inverse());

	      transform_tmp.setOrigin(tf::Vector3(0,0,0));
	      
	      br->sendTransform(tf::StampedTransform(transform_tmp, ros::Time::now(), sensor_base, sensor));
	      
	  }


        }
        catch (tf::TransformException ex)
        {
	  ROS_ERROR("%s",ex.what());
        }

        publish_accel(i,sensor_var[i][0],sensor_var[i][1],sensor_var[i][2]);
        publish_gyro(i,sensor_var[i][3],sensor_var[i][4],sensor_var[i][5]);
        publish_magn(i,sensor_var[i][9],sensor_var[i][10],sensor_var[i][11]);

        publish_sensor_type();
        publish_sensor_number(i);
    }
    
}
float get_angle(float num, float denom)
{
    if (atan2(num,denom)!=NAN)
        return atan2(num,denom);
    else
        return 0;
}

float get_magnitude(float x, float y, float z)
{
    return sqrt(x*x+y*y+z*z);
}
float get_theta(float iteration1, float iteration2,float time_interval)
{
    float ret;
    float alpha = 0.5;

	ret = (iteration2-iteration1)*time_interval; //14.375 LSB/(º/s) inverse value is 0.069565

    return ret;
}

void publish_accel(int n, float ax , float ay , float az)
{
    int x_pos = -10;
    int y_pos = 10*n;
    
    marker_id++;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id =marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = sensors_pos_y[n];
    marker.pose.position.z = sensors_pos_z[n];
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.2;
    marker.color.b = 0.2;	
    
    marker_id++;
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "/world";
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "my_namespace";
    marker2.id =marker_id;
    marker2.type = visualization_msgs::Marker::ARROW;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = x_pos;
    marker2.pose.position.y = sensors_pos_y[n];
    marker2.pose.position.z = sensors_pos_z[n];
    marker2.pose.orientation.x = 1.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 0.0;
    marker2.scale.x = 3.0;
    marker2.scale.y = 5.0;
    marker2.scale.z = 18.0*ax/2048+1;
    marker2.color.a = 1.0;
    marker2.color.r = 1.0;
    marker2.color.g = 0.0;
    marker2.color.b = 0.0;
    
    marker_id++;
    visualization_msgs::Marker marker3;
    marker3.header.frame_id = "/world";
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "my_namespace";
    marker3.id =marker_id;
    marker3.type = visualization_msgs::Marker::ARROW;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.pose.position.x = x_pos;
    marker3.pose.position.y = sensors_pos_y[n];
    marker3.pose.position.z = sensors_pos_z[n];
    marker3.pose.orientation.x = 1.0;
    marker3.pose.orientation.y = 1.0;
    marker3.pose.orientation.z = 0.0;
    marker3.pose.orientation.w = 0.0;
    marker3.scale.x = 3.0;
    marker3.scale.y = 3.0;
    marker3.scale.z = 18.0*ay/2048+1;
    marker3.color.a = 1.0;
    marker3.color.r = 0.0;
    marker3.color.g = 1.0;
    marker3.color.b = 0.0;
    
    marker_id++;
    visualization_msgs::Marker marker4;
    marker4.header.frame_id = "/world";
    marker4.header.stamp = ros::Time::now();
    marker4.ns = "my_namespace";
    marker4.id =marker_id;
    marker4.type = visualization_msgs::Marker::ARROW;
    marker4.action = visualization_msgs::Marker::ADD;
    marker4.pose.position.x = x_pos;
    marker4.pose.position.y = sensors_pos_y[n];
    marker4.pose.position.z = sensors_pos_z[n];
    marker4.pose.orientation.x = 1.0;
    marker4.pose.orientation.y = 0.0;
    marker4.pose.orientation.z = 1.0;
    marker4.pose.orientation.w = 0.0;
    marker4.scale.x = 3.0;
    marker4.scale.y = 3.0;
    marker4.scale.z = 18.0*az/2048+1;
    marker4.color.a = 1.0;
    marker4.color.r = 0.0;
    marker4.color.g = 0.0;
    marker4.color.b = 1.0;	
    
    vis_pub.publish(marker);
    vis_pub.publish( marker2 );
    vis_pub.publish( marker3 );
    vis_pub.publish( marker4 );
}

void publish_gyro(int n, float gx , float gy , float gz)
{
    int x_pos = 0;

	string sensor  = str( boost::format("/sensor_%d") % n );
    
    marker_id++;
    visualization_msgs::Marker marker;
    marker.header.frame_id = sensor;
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id =marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 3.0;
    marker.scale.y = 3.0;
    marker.scale.z = 0.6;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.2;
    marker.color.b = 0.2;	
    
    marker_id++;
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = sensor;
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "my_namespace";
    marker2.id =marker_id;
    marker2.type = visualization_msgs::Marker::ARROW;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = x_pos;
    marker2.pose.position.y = 0;
    marker2.pose.position.z = 0;
    marker2.pose.orientation.x = 1.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 0.0;
    marker2.scale.x = 3.0;
    marker2.scale.y = 5.0;
    marker2.scale.z = 20.0*gx/10000+1;
    marker2.color.a = 1.0;
    marker2.color.r = 1.0;
    marker2.color.g = 0.0;
    marker2.color.b = 0.0;
    
    marker_id++;
    visualization_msgs::Marker marker3;
    marker3.header.frame_id = sensor;
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "my_namespace";
    marker3.id =marker_id;
    marker3.type = visualization_msgs::Marker::ARROW;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.pose.position.x = x_pos;
    marker3.pose.position.y = 0;
    marker3.pose.position.z = 0;
    marker3.pose.orientation.x = 1.0;
    marker3.pose.orientation.y = 1.0;
    marker3.pose.orientation.z = 0.0;
    marker3.pose.orientation.w = 0.0;
    marker3.scale.x = 3.0;
    marker3.scale.y = 5.0;
    marker3.scale.z = 20.0*gy/10000+1;
    marker3.color.a = 1.0;
    marker3.color.r = 0.0;
    marker3.color.g = 1.0;
    marker3.color.b = 0.0;
    
    marker_id++;
    visualization_msgs::Marker marker4;
    marker4.header.frame_id = sensor;
    marker4.header.stamp = ros::Time::now();
    marker4.ns = "my_namespace";
    marker4.id =marker_id;
    marker4.type = visualization_msgs::Marker::ARROW;
    marker4.action = visualization_msgs::Marker::ADD;
    marker4.pose.position.x = x_pos;
    marker4.pose.position.y = 0;
    marker4.pose.position.z = 0;
    marker4.pose.orientation.x = 1.0;
    marker4.pose.orientation.y = 0.0;
    marker4.pose.orientation.z = 1.0;
    marker4.pose.orientation.w = 0.0;
    marker4.scale.x = 3.0;
    marker4.scale.y = 5.0;
    marker4.scale.z = 20.0*gz/10000+1;
    marker4.color.a = 1.0;
    marker4.color.r = 0.0;
    marker4.color.g = 0.0;
    marker4.color.b = 1.0;	
    
    vis_pub.publish(marker);
    vis_pub.publish( marker2 );
    vis_pub.publish( marker3 );
    vis_pub.publish( marker4 );
}

void publish_magn(int n, float mx , float my , float mz)
{
    int x_pos = 10;
    int y_pos = 10*n;
    
    marker_id++;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id =marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = sensors_pos_y[n];
    marker.pose.position.z = sensors_pos_z[n];
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.2;
    marker.color.b = 0.2;	
    
    marker_id++;
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "/world";
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "my_namespace";
    marker2.id =marker_id;
    marker2.type = visualization_msgs::Marker::ARROW;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = x_pos;
    marker2.pose.position.y = sensors_pos_y[n];
    marker2.pose.position.z = sensors_pos_z[n];
    marker2.pose.orientation.x = 1.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 0.0;
    marker2.scale.x = 3.0;
    marker2.scale.y = 5.0;
    marker2.scale.z = 10.0*mx/2048+1;
    marker2.color.a = 1.0;
    marker2.color.r = 1.0;
    marker2.color.g = 0.0;
    marker2.color.b = 0.0;	
    
    
    marker_id++;
    visualization_msgs::Marker marker3;
    marker3.header.frame_id = "/world";
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "my_namespace";
    marker3.id =marker_id;
    marker3.type = visualization_msgs::Marker::ARROW;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.pose.position.x = x_pos;
    marker3.pose.position.y = sensors_pos_y[n];
    marker3.pose.position.z = sensors_pos_z[n];
    marker3.pose.orientation.x = 1.0;
    marker3.pose.orientation.y = 1.0;
    marker3.pose.orientation.z = 0.0;
    marker3.pose.orientation.w = 0.0;
    marker3.scale.x = 3.0;
    marker3.scale.y = 5.0;
    marker3.scale.z = 10.0*my/2048+1;
    marker3.color.a = 1.0;
    marker3.color.r = 0.0;
    marker3.color.g = 1.0;
    marker3.color.b = 0.0;
    
    marker_id++;
    visualization_msgs::Marker marker4;
    marker4.header.frame_id = "/world";
    marker4.header.stamp = ros::Time::now();
    marker4.ns = "my_namespace";
    marker4.id =marker_id;
    marker4.type = visualization_msgs::Marker::ARROW;
    marker4.action = visualization_msgs::Marker::ADD;
    marker4.pose.position.x = x_pos;
    marker4.pose.position.y = sensors_pos_y[n];
    marker4.pose.position.z = sensors_pos_z[n];
    marker4.pose.orientation.x = 1.0;
    marker4.pose.orientation.y = 0.0;
    marker4.pose.orientation.z = 1.0;
    marker4.pose.orientation.w = 0.0;
    marker4.scale.x = 3.0;
    marker4.scale.y = 5.0;
    marker4.scale.z = 10.0*mz/2048+1;
    marker4.color.a = 1.0;
    marker4.color.r = 0.0;
    marker4.color.g = 0.0;
    marker4.color.b = 1.0;	
    
    vis_pub.publish(marker);
    vis_pub.publish( marker2 );
    vis_pub.publish( marker3 );
    vis_pub.publish( marker4 );
}

void publish_sensor_type(void)
{
    string accel = "Acelerometro";
    string gyro = "Giroscopio";
    string magn = "Magnetometro";
    
    marker_id++;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id =marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -10;
    marker.pose.position.y = 0;
    marker.pose.position.z = -5;
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = 1.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 1.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text =accel;
    
    marker_id++;
    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "/world";
    marker2.header.stamp = ros::Time();
    marker2.ns = "my_namespace";
    marker2.id =marker_id;
    marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = 0;
    marker2.pose.position.y = 0;
    marker2.pose.position.z = -5;
    marker2.pose.orientation.x = 1.0;
    marker2.pose.orientation.y = 1.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 0.0;
    marker2.scale.x = 0.5;
    marker2.scale.y = 0.5;
    marker2.scale.z = 1.5;
    marker2.color.a = 1.0;
    marker2.color.r = 0.0;
    marker2.color.g = 0.0;
    marker2.color.b = 0.0;
    marker2.text =gyro;
    
    marker_id++;
    visualization_msgs::Marker marker3;
    marker3.header.frame_id = "/world";
    marker3.header.stamp = ros::Time();
    marker3.ns = "my_namespace";
    marker3.id =marker_id;
    marker3.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.pose.position.x = 10;
    marker3.pose.position.y = 0;
    marker3.pose.position.z = -5;
    marker3.pose.orientation.x = 1.0;
    marker3.pose.orientation.y = 1.0;
    marker3.pose.orientation.z = 0.0;
    marker3.pose.orientation.w = 0.0;
    marker3.scale.x = 5.5;
    marker3.scale.y = 2.5;
    marker3.scale.z = 1.5;
    marker3.color.a = 1.0;
    marker3.color.r = 0.0;
    marker3.color.g = 0.0;
    marker3.color.b = 0.0;
    marker3.text =magn;
    
    vis_pub.publish(marker);
    vis_pub.publish(marker2);
    vis_pub.publish(marker3);
}

void publish_sensor_number(int n)
{
    string sens = "Sensor ";
    string nsensor;
    
    int sensors_pos_y[4] = {0 , 5 , 30 , 35};
    
    ostringstream convert;   // stream used for the conversion
    convert << n;      // insert the textual representation of 'Number' in the characters in the stream
    nsensor = sens + convert.str(); // set 'Result' to the contents of the stream
    
    
    
    int z_pos = -10;
    marker_id++;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id =marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -15;
    marker.pose.position.y = sensors_pos_y[n];
    marker.pose.position.z = z_pos;
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = 1.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 1.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text =nsensor;
    
    vis_pub.publish(marker);
}
/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "imu_rviz");
    n = new(ros::NodeHandle);
    br = new(tf::TransformBroadcaster);
    listener = new(tf::TransformListener);

    chatter_pub = n->advertise<imu_network::filtered_imu_network>("topic_filtered_imu", 1000);
    ros::Subscriber sub = n->subscribe("topic_filtered_imu", 1000, chatterCallback);

    
    vis_pub = n->advertise<visualization_msgs::Marker>( "marker_sensor", 0 );
        
    ros::spin();

    return 0;
}
