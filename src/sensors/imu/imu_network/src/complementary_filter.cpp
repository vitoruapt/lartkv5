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
 * @file complementary_filter.cpp
 * @author Telmo Rafeiro n.º 45349 (rafeiro@ua.pt)
 * @brief Complementary filter - Roll & Pitch calculation
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <math.h> 
#include <cmath>
#include <vector>

#include <boost/format.hpp> 

#include <imu_network/sensors_network.h>
#include <imu_network/filtered_imu_network.h>
#include <imu_network/filtered_imu.h>

#define MAX_MEAN_IT 25 //number of iterations to measure the mean values from gyroscopes

#define GYRO_CONVERT M_PI/180*0.069565 // conversion of gyro's values from ADC to rad/sec

using namespace std;


ros::NodeHandle* n;
ros::Publisher chatter_pub;

int sensors_number;
int first_run=0;

vector<float>gyro_angle[3];
vector<float>accel_ang_x;
vector<float>accel_ang_y;

vector<float>angle[3];
vector<float>angle_last_it[3];

vector<float>gyro_mean_vect[MAX_MEAN_IT][3]; //vectors for allocation of values of gyro for mean
vector<float>gyro_real_mean[3]; //vetor of gyro's means
vector<float>gyro_last_it[3];

float period = 1/7.35;
float alpha = 0.3;
float accel_alpha = 0.7;

int mean_it = 0;

imu_network::filtered_imu_network complementary_filter_network;

void chatterCallback(const imu_network::sensors_network::ConstPtr& msg) //callback that intercept the sensor's raw data
{
    float ax,ay,az;
    if(first_run ==0)
    {
        
        sensors_number=(int)msg->sensors_val[0].total_number;
        
        for (int i=0;i<sensors_number;i++)
        {
	  //published message initialization
	  imu_network::filtered_imu filtered_imu_data;
	  complementary_filter_network.filtered_imu_network.push_back(filtered_imu_data);
	  
	  ax = msg->sensors_val[i].S_Ax;
	  ay = msg->sensors_val[i].S_Ay;
	  az = msg->sensors_val[i].S_Az;
	  
	  accel_ang_x.push_back(atan(ay/sqrt((ax*ax)+(az*az))));
	  accel_ang_y.push_back(atan(ax/sqrt((ay*ay)+(az*az))));
	  
	  gyro_angle[0].push_back(0);
	  gyro_angle[1].push_back(0);
	  gyro_angle[2].push_back(0);
	  
	  gyro_last_it[0].push_back(0);
	  gyro_last_it[1].push_back(0);
	  gyro_last_it[2].push_back(0);
	  
	  angle_last_it[0].push_back(accel_ang_x[i]);
	  angle_last_it[1].push_back(accel_ang_y[i]);
	  angle_last_it[2].push_back(0);
	  
	  angle[0].push_back(0);
	  angle[1].push_back(0);
	  angle[2].push_back(0);
	  
        }
    
    first_run = 1;
    return;
    }
    
    //getting the gyro's means...
    if(mean_it<MAX_MEAN_IT)
    {
        for(int i= 0;i<sensors_number;i++)
        {
	  gyro_mean_vect[i][0].push_back(msg->sensors_val[i].S_Gx);
	  gyro_mean_vect[i][1].push_back(msg->sensors_val[i].S_Gy);
	  gyro_mean_vect[i][2].push_back(msg->sensors_val[i].S_Gz);
	  gyro_real_mean[0].push_back(i);
	  gyro_real_mean[1].push_back(i);
	  gyro_real_mean[2].push_back(i);
        }
        mean_it++;
        return;
    }
    //getting gyros's means to compensate it (if it's needed!)
    if(mean_it==MAX_MEAN_IT)
    {
        for(int i= 0;i<sensors_number;i++)
        {
	  for(int j=0;j<mean_it;j++)
	  {
	      gyro_real_mean[0][i]+=gyro_mean_vect[i][0][j];
	      gyro_real_mean[1][i]+=gyro_mean_vect[i][1][j];
	      gyro_real_mean[2][i]+=gyro_mean_vect[i][2][j];
	  }
	  gyro_real_mean[0][i]/=MAX_MEAN_IT;
	  gyro_real_mean[1][i]/=MAX_MEAN_IT;
	  gyro_real_mean[2][i]/=MAX_MEAN_IT;
	  cout<<"mean_it = "<<mean_it<<" i="<<i<<" gyro_real_mean(0)="<<gyro_real_mean[0][i]<<" gyro_real_mean(1)="<<gyro_real_mean[1][i]<<" gyro_real_mean(2)="<<gyro_real_mean[2][i]<<endl;
        }
        mean_it++;      
    }
    
    float sensor_var[sensors_number][9];
    
    for(int i = 0; i<sensors_number; i++)
    {
        //sensors_values -> variable changing
        sensor_var[i][0]=msg->sensors_val[i].S_Ax;
        sensor_var[i][1]=msg->sensors_val[i].S_Ay;
        sensor_var[i][2]=msg->sensors_val[i].S_Az;
        sensor_var[i][3]=msg->sensors_val[i].S_Mx;
        sensor_var[i][4]=msg->sensors_val[i].S_My;
        sensor_var[i][5]=msg->sensors_val[i].S_Mz;
        sensor_var[i][6]=msg->sensors_val[i].S_Gx;
        sensor_var[i][7]=msg->sensors_val[i].S_Gy;
        sensor_var[i][8]=msg->sensors_val[i].S_Gz;
        
        // getting Roll & Pitch from accelerometers
        ax = sensor_var[i][0];
        ay = sensor_var[i][1];
        az = sensor_var[i][2];
        
        accel_ang_x[i] = (accel_alpha * atan(ay/sqrt((ax*ax)+(az*az)))) + ((1-accel_alpha) * angle_last_it[0][i]);
        accel_ang_y[i] = (accel_alpha * atan(ax/sqrt((ay*ay)+(az*az)))) + ((1-accel_alpha) * angle_last_it[1][i]);
        
        angle_last_it[0][i]=accel_ang_x[i];
        angle_last_it[1][i]=accel_ang_y[i];
        
        // getting Roll , Pitch & Yaw from gyroscopes
        gyro_angle[0][i] = ((sensor_var[i][6]-gyro_real_mean[0][i])) * period * GYRO_CONVERT;
        gyro_angle[1][i] = ((sensor_var[i][7]-gyro_real_mean[1][i])) * period * GYRO_CONVERT;
        gyro_angle[2][i] = ((sensor_var[i][8]-gyro_real_mean[2][i])) * period * GYRO_CONVERT;
        
        float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
		angle[0][i] = (1-alpha) * (angle[0][i] + gyro_angle[0][i]) + alpha * accel_ang_x[i];
		angle[1][i] = (1-alpha) * (angle[1][i] + gyro_angle[1][i]) + alpha * accel_ang_y[i];
		angle[2][i] = angle[2][i] + gyro_angle[2][i];

		// setting algorithm results on message variable
        complementary_filter_network.filtered_imu_network[i].linear_acceleration_x = sensor_var[i][0];
        complementary_filter_network.filtered_imu_network[i].linear_acceleration_y = sensor_var[i][1];
        complementary_filter_network.filtered_imu_network[i].linear_acceleration_z = sensor_var[i][2];
        
        complementary_filter_network.filtered_imu_network[i].angular_velocity_x = sensor_var[i][6];
        complementary_filter_network.filtered_imu_network[i].angular_velocity_y = sensor_var[i][7];
        complementary_filter_network.filtered_imu_network[i].angular_velocity_z = sensor_var[i][8];
        
        complementary_filter_network.filtered_imu_network[i].angular_displacement_x = angle[0][i];
        complementary_filter_network.filtered_imu_network[i].angular_displacement_y = angle[1][i];
        complementary_filter_network.filtered_imu_network[i].angular_displacement_z = angle[2][i];
       
        complementary_filter_network.filtered_imu_network[i].magnetometer_x = sensor_var[i][3];
        complementary_filter_network.filtered_imu_network[i].magnetometer_y = sensor_var[i][4];
        complementary_filter_network.filtered_imu_network[i].magnetometer_z = sensor_var[i][5];

        complementary_filter_network.filtered_imu_network[i].number = i;
        complementary_filter_network.filtered_imu_network[i].total_number= sensors_number;
        
        complementary_filter_network.filtered_imu_network[i].algorithm = 1;
        
        complementary_filter_network.filtered_imu_network[i].header.stamp = msg->sensors_val[i].header.stamp;
        complementary_filter_network.filtered_imu_network[i].header.frame_id = "filtered_imu_network";
        
    }
    chatter_pub.publish(complementary_filter_network);
    
}

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "complementary_imu");
    n = new(ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("topic_raw_data", 1000, chatterCallback);
    chatter_pub = n->advertise<imu_network::filtered_imu_network>("topic_filtered_imu", 1000);
    
    ros::spin();
    
    return 0;
}
