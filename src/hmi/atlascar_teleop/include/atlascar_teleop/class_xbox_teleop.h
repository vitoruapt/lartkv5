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
 * @file class_xbox_teleop.h
 * @brief class_xbpox_teleop.h file for this node. Includes, class global vars, class methods, etc.
 */

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
// #include <atlascar_base/AtlascarCommand.h>



#ifndef _ATLASCAR_XBOX_TELEOP_
#define _ATLASCAR_XBOX_TELEOP_




using namespace std;
using namespace ros;


/** 
 * @brief 
 */
class XboxTeleopAtlascar
{
	public:
		/** 
		 * @brief constructor
		 * 
		 * @param num_but - number of digital buttons in the gamepad
		 * @param num_axis - number of analog buttons or axis
		 */
		XboxTeleopAtlascar(int num_but, int num_axis);
		/** 
		 * @brief destructor
		 */
		~XboxTeleopAtlascar();
		
		///struct to allow override the cout
		struct TYPE_analog{
			int toggled;
			float val;

			friend ostream& operator<<(ostream &o, const TYPE_analog& i)
			{
					o<<"toogle:"<<i.toggled<<"\nval:"<<i.val<<endl;
				return o;
			}
		};

		///digital buttons at xbox and their function
		enum{MAN_AUTO_STEER=0, MAN_AUTO_BRAKE=1, MAN_AUTO_THROTTLE=2, MAN_AUTO_CLUTCH=3, _B4=4, MAN_AUTO_IGNITION=5,\
			START=6, E_STOP=7, _B8=8, _B9=9, HIGH_LIGHTS=10, WARNING_LIGHTS=11, MEDIUM_LIGHTS=12,\
			_B13=13, STOP=14};
		///analog buttons and their functionalities
		enum{STEERING=0, _A1=1, BRAKE=2, _A3=3, CLUTCH=4, THROTTLE=5};


	private:
		///constructor
		void XboxCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_teleop;
		///vector of digital buttons values
		vector<int> button;
		///vector of analog buttons values
		vector<TYPE_analog> analog;

		///minimum & maximum steering angle values
		double steering_max,steering_min;
		///minimum & maximum brake values
		double brake_min,brake_max;
		///minimum & maximum clutch values
		double clutch_min,clutch_max;
		///minimum & maximum throttle values
		double throttle_min,throttle_max;
		
		
		///values to create the linear interpolation
		double steering_left_ang, steering_right_ang;
		///values to create the linear interpolation
		double m_left_ang, b_left_ang, m_right_ang, b_right_ang;

		///values to create the linear interpolation
		double speed_min, speed_max;
		///values to create the linear interpolation
		double m_speed, b_speed;
		///values to create the linear interpolation
		double m_brake, b_brake;
		///specifies the number of digital buttons and analog buttons in the gamepad
		int num_buttons, num_axes;
		
		///atlascar command message
// 		atlascar_base::AtlascarCommand command;
		ros::Publisher atlascar_cmd_;
		ros::Subscriber joy_sub_;
		ros::Publisher ptu_pub;
};


#endif
