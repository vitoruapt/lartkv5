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
/**@file class_xbox_ptu_teleop.h
 * @brief class class_xbox_ptu_teleop header
 */

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <atlascar_base/AtlascarCommand.h>


#ifndef _ATLASCAR_XBOX_TELEOP_
#define _ATLASCAR_XBOX_TELEOP_

using namespace std;
using namespace ros;


/** 
 * @brief This class simplifies the implementation of a interface between xbox and PTU
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
		
		///struct to define a analog button
		struct TYPE_analog{
			int toggled;
			float val;

			friend ostream& operator<<(ostream &o, const TYPE_analog& i)
			{
					o<<"toogle:"<<i.toggled<<"\nval:"<<i.val<<endl;
				return o;
			}
		};

		///enumerator to describe all digital buttons
		enum{MAN_AUTO_STEER=0, MAN_AUTO_BRAKE=1, MAN_AUTO_THROTTLE=2, MAN_AUTO_CLUTCH=3, _B4=4, MAN_AUTO_IGNITION=5,\
			START=6, E_STOP=7, _B8=8, _B9=9, HIGH_LIGHTS=10, WARNING_LIGHTS=11, MEDIUM_LIGHTS=12,\
			_B13=13, STOP=14};
		///enumerator to describe all analog buttons
		enum{STEERING=0, _A1=1, BRAKE=2, _A3=3, CLUTCH=4, THROTTLE=5};


	private:
		
		/** 
		 * @brief callback that is called when a xbox msg is received
		 * 
		 * @param joy - struct with gamepad message
		 */
		void XboxCallback(const sensor_msgs::Joy::ConstPtr& joy);

		/** 
		 * @brief 
		 */
		ros::NodeHandle nh_teleop;

		/** 
		 * @brief vector with all digital buttons status
		 */
		vector<int> button;

		/** 
		 * @brief  vector with analog/axis values
		 */
		vector<TYPE_analog> analog;

		/** 
		 * @brief limit values for the steering angles
		 */
		float steering_left_ang, steering_right_ang;
		/** 
		 * @brief line segment slope, left side and right side
		 */
		float m_left_ang, b_left_ang, m_right_ang, b_right_ang;

		/** 
		 * @brief car speed limits
		 */
		float speed_min, speed_max;
		/** 
		 * @brief line segment slope to calculate whished car speed value
		 */
		float m_speed, b_speed;
		/** 
		 * @brief line segment slope to calculate whished car speed value
		 */
		float m_brake, b_brake;
		/** 
		 * @brief variable with number of digital buttons and analog axis
		 */
		int num_buttons, num_axes;
		
		/** 
		 * @brief struct with the Atlascar Base Command message
		 */
		atlascar_base::AtlascarCommand command;
		/** 
		 * @brief method just to publish the base message
		 */
		ros::Publisher atlascar_cmd_;
		/** 
		 * @brief method to subscribe the gamepad message
		 */
		ros::Subscriber joy_sub_;

};


#endif
