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
 * \brief Node that publishes commands from a gamepad
 */

#include <iostream>
#include "ros/ros.h"

#include <signal.h>

#include <atlasmv_base/AtlasmvMotionCommand.h>
#include <atlasmv_base/c_atlasmv.h>

#include <atlasmv_base/class_gamepad.h>
#include <math.h>

using namespace ros;
using namespace std;

atlasmv_base::AtlasmvMotionCommand command;
Publisher commandPublisherPtr;
bool in_reverse=false;
int last_value=-32768.;
Publisher commandPublisher;


void GamepadSearch(int value,void*parameters)
{
	atlasmv_base::AtlasmvMotionCommand command_local;

	if(value==1)
	{
		command_local.dir=0;
		command_local.speed=0.2;
		command_local.lifetime=INFINITY;
		command_local.priority=1;
		command_local.header.stamp=ros::Time::now();
		commandPublisher.publish(command_local);
	}
	else
	{
		command_local.dir=0;
		command_local.speed=0.0;
		command_local.lifetime=0.1;
		command_local.priority=1;
		command_local.header.stamp=ros::Time::now();
		commandPublisher.publish(command_local);
	}
	
}


void GamepadSpeed(int value,void*userdata)
{
	TYPE_atlasmv_public_params*p=(TYPE_atlasmv_public_params*)userdata;
	
	double speed=(((double)(value)/32768. + 1)/2);
	last_value=value;
	
	if(!in_reverse)
		speed=speed*p->max_forward_speed;
	else
		speed=speed*p->max_backward_speed;
	
	cout<<"speed: "<<speed<<endl;
	command.speed=speed;
	command.lifetime=1;//INFINITY;
	command.priority=3;
	command.header.stamp=ros::Time::now();
	commandPublisher.publish(command);
}

void GamepadReverse(int value,void*parameters)
{
	if(value)
		in_reverse=1;
	else
		in_reverse=0;
	command.priority=3;
	GamepadSpeed(last_value,parameters);
}

void GamepadDir(int value,void*userdata)
{
	TYPE_atlasmv_public_params*p=(TYPE_atlasmv_public_params*)userdata;
	
	double dir=((double)(-value)/32768.);
	
	if(dir>0)
		dir*=fabs(p->maximum_dir);
	else
		dir*=fabs(p->minimum_dir);

	command.dir=dir;
// 	command.dir=0.0;
	command.lifetime=INFINITY;
	command.priority=3;
	command.header.stamp=ros::Time::now();
	commandPublisher.publish(command);
}

void GamepadZeroSteering(int value,void*userdata)
{
	command.dir=0;
	command.lifetime=INFINITY;
	command.priority=3;
	command.header.stamp=ros::Time::now();
	commandPublisher.publish(command);
}

void GamepadDisconnect(int value,void*userdata)
{
	command.dir=0;
	command.speed=0;
	command.lifetime=0.2;
	command.priority=3;
	command.header.stamp=ros::Time::now();
	commandPublisher.publish(command);
}

int main(int argc,char**argv)
{
	TYPE_atlasmv_public_params parameters;
	
	// Initialize ROS
	init(argc,argv,"atlasmv_command_publisher");
	
	NodeHandle nh("~");
	
	class_gamepad gamepad;
	
	char*dev=(char*)malloc(1024*sizeof(char));
	std::string dev_s;
	
	nh.param("maximum_dir", parameters.maximum_dir,0.355);
	nh.param("minimum_dir", parameters.minimum_dir,-0.38);
	nh.param("max_forward_speed", parameters.max_forward_speed,4.0);
	nh.param("max_backward_speed", parameters.max_backward_speed,-1.);
	nh.param("device_gamepad",dev_s,(std::string)"/dev/input/js0");
	
	strcpy(dev,dev_s.c_str());
	
	cout<<endl<<"Gamepad device: "<<dev_s<<endl;
	int ret=gamepad.StartComm(dev);
	gamepad.plerr(ret);
	
	//Search mode (advance at low speed with zero steering)
	ret=gamepad.RegisterCallback(gamepad.BUTTON,3,GamepadSearch,&parameters);
	gamepad.plerr(ret);
	
	//Reverse mode
	ret=gamepad.RegisterCallback(gamepad.BUTTON,0,GamepadReverse,&parameters);
	gamepad.plerr(ret);
	
	//Forward motion of the atlas robots
	ret=gamepad.RegisterCallback(gamepad.AXIS,5,GamepadSpeed,&parameters);
	gamepad.plerr(ret);
	
	//Forward motion of the atlas robots
	ret=gamepad.RegisterCallback(gamepad.BUTTON,6,GamepadDisconnect,&parameters);
	gamepad.plerr(ret);
	
	//Forward motion of the atlas robots
	ret=gamepad.RegisterCallback(gamepad.BUTTON,7,GamepadDisconnect,&parameters);
	gamepad.plerr(ret);
	
	//Forward motion of the atlas robots
	ret=gamepad.RegisterCallback(gamepad.BUTTON,2,GamepadZeroSteering,&parameters);
	gamepad.plerr(ret);
	
	//Direction of the atlas robots
	ret=gamepad.RegisterCallback(gamepad.AXIS,0,GamepadDir,&parameters);
	gamepad.plerr(ret);
	
	/*Publisher*/ commandPublisher = nh.advertise<atlasmv_base::AtlasmvMotionCommand>("/atlasmv/base/motion", 1000);
	commandPublisherPtr=commandPublisher;
	
	command.lifetime=INFINITY;
	
	
	Rate r(50);//Hz
	
	while(ros::ok())//quits on SIGINT
	{
		gamepad.Dispatch(0);
// 		cout<<command<<endl;
		spinOnce();	
		r.sleep();
	}
}
