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
 * @file kalman_imu_data.cpp
 * @author Telmo Rafeiro n.º 45349 (rafeiro@ua.pt)
 * @brief Extended Kalman Filter algorithm implementation only with gyroscopes data
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
#include <kfilter/ekfilter.hpp>
#include <kfilter/kvector.hpp>

//messages 

#include <imu_network/sensors_network.h>
#include <imu_network/filtered_imu_network.h>
#include <imu_network/filtered_imu.h>


#define MAX_MEAN_IT 64 //number of iterations to measure the mean values from gyroscopes

#define GYRO_CONVERT M_PI/180*0.069565 // conversion of gyro's values from ADC to rad/sec
using namespace std;

ros::NodeHandle* n;
ros::Publisher chatter_pub;


class imuEKF_sp : public Kalman::EKFilter<double,0,false,false,false>
{
    public:
        
        imuEKF_sp() 
        {
	  setDim(6, 0, 6, 3, 3);
	  Period = 0.025;
        }
        
    protected:
        
        void makeA()
        {
	  for (int i = 0 ; i<6 ; i++)
	  {
	      for(int j = 0 ; j<6 ; j++)
	      {
		if(i==j) A(i,j)=1.0;
		else A(i,j) = 0.0;
	      }
	  }
	  A(0,1) = Period;
	  A(2,3) = Period;
	  A(4,5) = Period;
	  
        }
        void makeH()
        {
	  for(int i=0;i<3;i++)
	  {
	      for(int j=0;j<6;j++)
	      {
		H(i,j)=0.0;
	      }
	  }
	  H(0,1)=1.0;
	  H(1,3)=1.0;
	  H(2,5)=1.0;
	  
        }
        void makeV()
        {
	  for(int i=0;i<3;i++)
	  {
	      for(int j=0;j<3;j++)
	      {
		if(i==j) V(i,j)=1.0;
		else V(i,j)=0.0;
	      }
	  }
	  
        }
        void makeR()
        {
	  for(int i=0;i<3;i++)
	  {
	      for(int j=0;j<3;j++)
	      {
		if(i==j) R(i,j)=75*M_PI/180; // 75 dps in rad/s
		    else R(i,j)=0.0;
	      }
	  }
	  
        }
        void makeW()
        {
	  
	  for(int i=0;i<6;i++)
	  {
	      for(int j=0;j<6;j++)
	      {
		if(i==j) W(i,j) = 1.0;
		else W(i,j) = 0.0;
	      }
	  }
	  
        }
        void makeQ()
        {
	  for(int i=0;i<6;i++)
	  {
	      for(int j=0;j<6;j++)
	      {
		if(i==j) Q(i,j) = 1.0;
		else Q(i,j) = 0.0;
	      }
	  }
	  
        }
        void makeProcess()
        {
	  Vector x_(x.size());
	  x_(0) = x(0) + x(1)*Period;
	  x_(1) = x(1);
	  x_(2) = x(2) + x(3)*Period;
	  x_(3) = x(3);
	  x_(4) = x(0) + x(5)*Period;
	  x_(5) = x(5);
	  
	  x.swap(x_);
        }
        void makeMeasure()
        {
	  Vector z(3);
	  z(0)=x(1);
	  z(1)=x(3);
	  z(2)=x(5);
        }
        
        double Period;
        
};

vector<imuEKF_sp> kalman_imu;

imuEKF_sp::Vector _z(3);
imuEKF_sp::Vector _u;

vector<ros::Time>time_stamp;
vector<float>gyro_valx;
vector<float>gyro_valy;
vector<float>gyro_valz;

vector<float>gyro_mean_vect[MAX_MEAN_IT][3]; //vectors for allocation of values of gyro for mean
vector<float>gyro_real_mean[3]; //vetor of gyro's means



int sensors_number;
int first_run=0;
int marker_id;
int mean_it = 0;

imu_network::filtered_imu_network kalman_data_network;

void chatterCallback(const imu_network::sensors_network::ConstPtr& msg)
{
    if(first_run ==0)
    {
        
        sensors_number=(int)msg->sensors_val[0].total_number;
        
        for (int i=0;i<sensors_number;i++)
        {
	  //published message initialization
	  imu_network::filtered_imu kalman_imu_data;
	  kalman_data_network.filtered_imu_network.push_back(kalman_imu_data);
	  
	  time_stamp.push_back(msg->sensors_val[i].header.stamp);
	  gyro_valx.push_back(msg->sensors_val[i].S_Gx*GYRO_CONVERT);
	  gyro_valy.push_back(msg->sensors_val[i].S_Gy*GYRO_CONVERT);
	  gyro_valz.push_back(msg->sensors_val[i].S_Gz*GYRO_CONVERT);

	  imuEKF_sp kalman_initializer;
	  kalman_imu.push_back(kalman_initializer);
	  
	  imuEKF_sp::Vector _x(6);
	  
	  imuEKF_sp::Matrix _P0(6,6);
	  
	  _x(0)=0.0;
	  _x(1)=0.0;
	  _x(2)=0.0;
	  _x(3)=0.0;
	  _x(4)=0.0;
	  _x(5)=0.0;
	  
	  for (int a = 0; a<6;a++)
	  {
	      for (int b=0;b<6;b++)
	      {
		if (a==b) _P0(a,b)=10.3;
		else _P0(a,b)=0.0;
	      }
	  }
	  kalman_imu[i].init(_x,_P0);
        }
        first_run = 1 ;  
        return;
    }
    
    imuEKF_sp::Vector _x(6);
    
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
    
    for(int i = 0;i<sensors_number;i++)
    {
        sensor_var[i][0]=msg->sensors_val[i].S_Ax;
        sensor_var[i][1]=msg->sensors_val[i].S_Ay;
        sensor_var[i][2]=msg->sensors_val[i].S_Az;
        sensor_var[i][3]=msg->sensors_val[i].S_Mx;
        sensor_var[i][4]=msg->sensors_val[i].S_My;
        sensor_var[i][5]=msg->sensors_val[i].S_Mz;
        sensor_var[i][6]=msg->sensors_val[i].S_Gx;
        sensor_var[i][7]=msg->sensors_val[i].S_Gy;
        sensor_var[i][8]=msg->sensors_val[i].S_Gz;
        
        _z(0)=(msg->sensors_val[i].S_Gx-gyro_real_mean[0][i])*GYRO_CONVERT;
        _z(1)=(msg->sensors_val[i].S_Gy-gyro_real_mean[1][i])*GYRO_CONVERT;
        _z(2)=(msg->sensors_val[i].S_Gz-gyro_real_mean[0][i])*GYRO_CONVERT;
        
        kalman_imu[i].step(_u,_z);
        
        imuEKF_sp::Vector result(6);
        result = kalman_imu[i].getX();
    //storing the values inside message variables...
    kalman_data_network.filtered_imu_network[i].linear_acceleration_x = sensor_var[i][0];
    kalman_data_network.filtered_imu_network[i].linear_acceleration_y = sensor_var[i][1];
    kalman_data_network.filtered_imu_network[i].linear_acceleration_z = sensor_var[i][2];
       
    kalman_data_network.filtered_imu_network[i].angular_velocity_x = result(1);
    kalman_data_network.filtered_imu_network[i].angular_velocity_y = result(3);
    kalman_data_network.filtered_imu_network[i].angular_velocity_z = result(5);
        
    kalman_data_network.filtered_imu_network[i].angular_displacement_x = result(0);
    kalman_data_network.filtered_imu_network[i].angular_displacement_y = result(2);
    kalman_data_network.filtered_imu_network[i].angular_displacement_z = result(4);
        
    kalman_data_network.filtered_imu_network[i].magnetometer_x = sensor_var[i][3];
    kalman_data_network.filtered_imu_network[i].magnetometer_y = sensor_var[i][4];
    kalman_data_network.filtered_imu_network[i].magnetometer_z = sensor_var[i][5];
        
    kalman_data_network.filtered_imu_network[i].number= i;
    kalman_data_network.filtered_imu_network[i].total_number=sensors_number;
        
    kalman_data_network.filtered_imu_network[i].algorithm = 0;
    
    kalman_data_network.filtered_imu_network[i].header.stamp = msg->sensors_val[i].header.stamp;
    kalman_data_network.filtered_imu_network[i].header.frame_id = "filtered_imu_network";
        
    }
    chatter_pub.publish(kalman_data_network);
    
}

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "kalman_imu_raw");
    n = new(ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("topic_raw_data", 1000, chatterCallback);
    chatter_pub = n->advertise<imu_network::filtered_imu_network>("topic_filtered_imu", 1000);
    
    ros::spin();

    return 0;
}
