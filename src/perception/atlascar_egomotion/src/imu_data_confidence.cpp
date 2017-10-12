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
 * \brief IMU data confidence calculation
 */
#include <sensor_msgs/Imu.h>
// #include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
// #include <atlascar_base/AtlascarStatus.h>
#include <std_msgs/Float64.h>
// #include <atlascar_base/AtlascarVelocityStatus.h>
using namespace std;
using namespace ros;


//
std_msgs::Float64 acc_norm_imu,tan_msg,cent_msg;
//
ros::Publisher* p_pitch_pub;
nav_msgs::Odometry odometry_;
sensor_msgs::Imu imu_data;
// atlascar_base::AtlascarStatus base_status;
// atlascar_base::AtlascarVelocityStatus base_velocity;
ros::Time t_i, t_i_1;
double vl_i,vl_i_1;
bool bool_init=true;
double v_ant;
void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg )
{
	imu_data=*msg;
}

// void VelocityMessageHandler(const atlascar_base::AtlascarVelocityStatus& msg)
// {
// 	base_velocity=msg;  
// 	if(bool_init)
//        {
//               t_i_1=msg.header.stamp;
//               t_i=msg.header.stamp;
//               bool_init=false;
//        }else
//        {
//               t_i_1=t_i;
//               t_i=msg.header.stamp;
//       }
// }
// void StatusMessageHandler(const atlascar_base::AtlascarStatus& msg)
// {
// 	base_status=msg;
// }
//this node just completes the information necessary to robot_pose_ekf
int main(int argc, char** argv)
{

       ros::init(argc, argv, "imu_egomotion");
       ros::NodeHandle n;

       Subscriber imu= n.subscribe("/atc/imu/data",1, ImuCallback);
//        Subscriber subscribeStatusMessages = n.subscribe ("/vhc/plc/status", 1, StatusMessageHandler);
//        Subscriber subscribeVelocityStatusMessages = n.subscribe ("/vhc/velocity/status", 1, VelocityMessageHandler);

	Publisher imu_norm=n.advertise<std_msgs::Float64>("/imu_norm",50);
	Publisher accel_tan=n.advertise<std_msgs::Float64>("/accel_tan",50);
	Publisher accel_cent=n.advertise<std_msgs::Float64>("/accel_cent",50);

           ros::Rate r(20.0);

       ROS_INFO("Starting to spin ...");
	double vl,phi;
		double phi_tmp=0;
		
       while(n.ok())
       {
           ROS_ERROR("Message type changed, AtlascarStatus and AtlascarVelocityStatus no longer avaliable!! please correct");
       
              spinOnce();   
              r.sleep();

          
//               vl=base_velocity.velocity;
//               phi=base_status.steering_wheel;
              
              if (phi ==0)
                     phi=phi_tmp;
              else
                     phi_tmp=phi;   
              
              
		//CALIBRATING STEERING WHEEL             
              //phi correction
              phi= phi +0.275;

		if (phi <=0)
		{
			phi = phi*0.74;
			phi=phi*-1;
		}
		else if (phi>0)
		{
			phi= phi*0.825;
			phi=phi*-1;
		}
	
		//corrrection velocity car like robot
		double w=1.27;
		double l=2.6;
		double instant_radious=fabs(l/tan(phi));
		double v_corrected;
		if(phi>=0)
			 v_corrected=vl*(instant_radious/(instant_radious-w/2));
		else	
			 v_corrected=vl*(instant_radious/(instant_radious+w/2));
		
		cout << "vl_ant" << v_ant << "\n"<< endl;
		cout << "vl" << vl << "\n"<< endl;
		cout << "vl_corre" << v_corrected << "\n"<< endl;
	//inst. radious
	//aceleração centripeta
	double dt = (t_i - t_i_1).toSec();
	double acc_cent=(v_corrected*v_corrected)/instant_radious;
	//cout << "acc_cent" << acc_cent << "\n"<< endl;
// 	double acc_tan=(v_corrected-v_ant)/dt;		
	//cout << "acc_tan" << acc_tan << "\n"<< endl;
	v_ant=v_corrected;	
		//tan_msg.data=acc_tan;
		tan_msg.data=dt;
		accel_tan.publish(tan_msg);

		cent_msg.data=acc_cent;
		accel_cent.publish(cent_msg);
	//imu data
	double accel_norm= sqrt((imu_data.linear_acceleration.x * imu_data.linear_acceleration.x) + (imu_data.linear_acceleration.y * imu_data.linear_acceleration.y) + (imu_data.linear_acceleration.z * imu_data.linear_acceleration.z));

		//cout << "accel_norm" << accel_norm-9.81 << endl;
		acc_norm_imu.data=accel_norm;
		imu_norm.publish(acc_norm_imu);


	}
       return 0;
}
