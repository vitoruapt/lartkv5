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
 * @file
 * @brief The main file, implements a simple algorithm to calculate the vehicle egomotion
 * @author Pedro Salvado
 */
#include <atlascar_egomotion/median_filter.h>
optoelectric::sensor_sharp_msg sharp_msg;
vector<double> measures;
bool new_status_message;

/** 
 * @brief SensorCallback
 * @details Subscribe optoeletric values
 * @param const optoelectric::sensor_sharp_msg &msg
 * @return void
*/
void SensorCallback (const optoelectric::sensor_sharp_msg &msg)
{
	//ROS_INFO("sensor_sharp_msg_received");
	sharp_msg=msg;
	new_status_message=true;

}
/** 
 * @brief main
 * @details Calculates roll, pitch angles and mean height and publishes respective transformation
*/
int main(int argc,char**argv)
{

	ros::init(argc, argv, "sensor_sharp_transformation_node");
	ros::NodeHandle n;
	ros::Subscriber sensor_msg_subs= n.subscribe("/snr/opto/sensor",1, SensorCallback);
	ros::Rate r(60.0);	

	int count_f_l=0;
	int count_f_r=0;
	int count_b_l=0;
	int count_b_r=0;
	float d_front_left_array[8];
	float d_front_right_array[8];
	float d_back_left_array[8];
	float d_back_right_array[8];
	while(n.ok())
	{
		spinOnce();   
		r.sleep();
		if(!new_status_message)
			continue;

		new_status_message=false;


		float dist_front_left=Median_Filter(count_f_l,sharp_msg.d_front_left,d_front_left_array);
		count_f_l++;

		float dist_front_right=Median_Filter(count_f_r,sharp_msg.d_front_right,d_front_right_array);
		count_f_r++;
	
		float dist_back_left=Median_Filter(count_b_l,sharp_msg.d_back_left,d_back_left_array);
		count_b_l++;
	
		float dist_back_right=Median_Filter(count_b_r,sharp_msg.d_back_right,d_back_right_array);
		count_b_r++;


		float z_mean=(dist_front_left + dist_front_right + dist_back_left + dist_back_right)/4;
		float roll_front=atan2((dist_front_left - dist_front_right),1.27);
		float roll_back=atan2((dist_back_left - dist_back_right),0.67);

		float pitch=atan2((dist_back_left +  dist_back_right)*0.5 - (dist_front_left + dist_front_right)*0.5,1.39*2);
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0.0, 0.0, z_mean) );
		transform.setRotation(tf::createQuaternionFromRPY((roll_front+roll_back)*0.5, pitch, 0) );
		br.sendTransform(tf::StampedTransform(transform, sharp_msg.header.stamp,"/ground", "/atc/vehicle/center_car_axis" ));

	}        

        return 0;
}
 
