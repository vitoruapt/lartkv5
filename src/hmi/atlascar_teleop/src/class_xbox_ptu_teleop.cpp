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
/**@file class_xbox_ptu_teleop.cpp
 * @brief class class_xbox_ptu_teleop source code that implements major part of communication code for atlascar teleop node
 */



#include <atlascar_teleop/class_xbox_teleop.h>


/** 
 * @brief function to initialize the XboxTeleop for the AtlasCAR robot
 * 
 * @param num_but - specifies the number of digital buttons that the xbox gamepad have
 * @param num_axis - specifies the number of analog buttons that the xbox gamepad have
 */
XboxTeleopAtlascar::XboxTeleopAtlascar(int num_but, int num_axis)
{
	num_buttons = num_but;
	num_axes = num_axis;

	button.resize(num_but);
	analog.resize(num_axes);
	
	steering_left_ang = 25.;
	steering_right_ang = 22.;


	m_left_ang = steering_left_ang;
	m_right_ang = steering_right_ang;

	b_left_ang = 0.0;
	b_right_ang = 0.0;

	speed_min = 0.0;
	speed_max = 1.0;

	m_speed = -(speed_max-speed_min)/2;
	b_speed = 0.5;
	
	m_brake = -0.5;
	b_brake = 0.5;

	for(uint i=0;i<button.size(); i++)
	{
		button[i] = 0;
	}
	
	for(uint i=0;i<analog.size(); i++)
	{
		analog[i].toggled = 0;
	}

	analog[BRAKE].val = 1.0;
	analog[THROTTLE].val = 1.0;
	analog[STEERING].val = 0.0;
	analog[CLUTCH].val = 0.0;

	atlascar_cmd_ = nh_teleop.advertise<atlascar_base::AtlascarCommand>("/cmd_out", 1);
	joy_sub_ = nh_teleop.subscribe<sensor_msgs::Joy>("/joy", 10, &XboxTeleopAtlascar::XboxCallback, this);

};

/** 
 * @brief private method associated to the callback when a new message from the gamepad is received, and construct the message that will be sent to atlascar_base
 * 
 * @param joy - is the message received from the gamepad
 */
void XboxTeleopAtlascar::XboxCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	

	//for the digital  buttons
	for(int i=0; i<num_buttons; i++)
	{
		if(	joy->buttons[i])
		{
			button[i] = !button[i];
		}
	}
	//for the analog  buttons
	for(int i=0; i<num_axes; i++)
	{
		if(!analog[i].toggled && ( fabs(joy->axes[i]) > 0.005 ))
			analog[i].toggled = 1;

		if( fabs(analog[i].val -joy->axes[i]) > 0.005 && analog[i].toggled)
		{
			analog[i].val = joy->axes[i];
		}
	} 

	command.auto_brake = button[MAN_AUTO_BRAKE];
	command.auto_clutch = button[MAN_AUTO_CLUTCH];
	command.auto_direction = button[MAN_AUTO_STEER];
	command.auto_throttle = button[MAN_AUTO_THROTTLE];
	command.auto_ignition = button[MAN_AUTO_IGNITION];
	
	command.emergency = button[E_STOP];
	command.lights_high = button[HIGH_LIGHTS];
	command.lights_medium = button[MEDIUM_LIGHTS];
	command.lights_minimum = button[MEDIUM_LIGHTS];
	command.ignition = button[START];
	if(button[STOP])
	{
		button[START]=0;
		command.ignition = 0;
	}

	if(analog[STEERING].val>0)
		command.steering_wheel = analog[STEERING].val*m_right_ang + b_right_ang;
	else if(analog[STEERING].val<0)
	{
		command.steering_wheel = analog[STEERING].val*m_left_ang + b_left_ang;
	}
	
	if(analog[CLUTCH].val>0)
	{
		command.clutch = analog[CLUTCH].val;
	}else
		command.clutch = 0.0;
		
	
	command.brake = analog[BRAKE].val*m_brake+b_brake;
	command.throttle = analog[THROTTLE].val*m_speed + b_speed;
	command.speed = 0.0;
	command.direct_control = 1;

	atlascar_cmd_.publish(command);

};


