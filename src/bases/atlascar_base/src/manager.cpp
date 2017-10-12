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
\file
\brief Manager class implementation
*/

#include "manager.h"

namespace atlascar_base
{

Manager::Manager(const ros::NodeHandle& nh)
:nh_(nh)
{}

Manager::~Manager()
{}

void Manager::init()
{
	safety_command_.reset(new atlascar_base::ManagerCommand);
	
	//Set the safety message default values, this messages will never be removed from the list
	safety_command_->lifetime=INFINITY;
	safety_command_->priority=0;
	
    current_state = INIT;
    
	//Push the safety message into the queue
	command_queue_.push_msg(safety_command_);
}

void Manager::setupMessaging()
{
	command_sub_ = nh_.subscribe("command", 1, &Manager::commandCallback, this);
	
	status_plc_sub_ = nh_.subscribe("plc_status", 1, &Manager::plcStatusCallback, this);
	status_driver_sub_ = nh_.subscribe("driver_status", 1, &Manager::driverStatusCallback, this);
	status_odometer_sub_ = nh_.subscribe("odometer_status", 1, &Manager::odometerStatusCallback, this);
	status_gearbox_sub_ = nh_.subscribe("gearbox_status", 1, &Manager::gearboxStatusCallback, this);
	status_throttle_sub_ = nh_.subscribe("throttle_status", 1, &Manager::throttleStatusCallback, this);
	
	status_pub_ = nh_.advertise<atlascar_base::ManagerStatus>("status", 1);
	
	command_plc_pub_ = nh_.advertise<atlascar_base::PlcCommand>("plc_command", 1);
	
	command_gearbox_pub_ = nh_.advertise<atlascar_base::GearboxCommand>("gearbox_command", 1);
	command_throttle_pub_ = nh_.advertise<atlascar_base::ThrottleCommand>("throttle_command", 1);
}

void Manager::toOutgoingMessages(const atlascar_base::ManagerCommandPtr& command)
{
	gearbox_command_.gear=command->gear;
	gearbox_command_.priority=command->priority;
	gearbox_command_.lifetime=command->lifetime;

	gearbox_command_.header=command->header;
	
	plc_command_.clutch=command->clutch;
	plc_command_.brake=command->brake;
	plc_command_.steering_wheel=command->steering_wheel;
	plc_command_.rpm=0;
	plc_command_.lights_high=command->lights_high;
	plc_command_.lights_medium=command->lights_medium;
	plc_command_.lights_minimum=0;
	plc_command_.lights_left=command->lights_left;
	plc_command_.lights_right=command->lights_right;
	plc_command_.lights_brake=command->lights_brake;
	plc_command_.lights_reverse=command->lights_reverse;
	plc_command_.lights_warning=command->lights_warning;
	plc_command_.ignition=command->ignition;
	plc_command_.emergency=command->emergency;
	plc_command_.auto_ignition=command->auto_ignition;
	plc_command_.auto_brake=command->auto_brake;
	plc_command_.auto_direction=command->auto_direction;
	plc_command_.auto_clutch=command->auto_clutch;
	
	plc_command_.priority=command->priority;
	plc_command_.lifetime=command->lifetime;

	plc_command_.header=command->header;
		
	throttle_command_.throttle=command->throttle;
	throttle_command_.mode=command->auto_throttle;
	throttle_command_.priority=command->priority;
	throttle_command_.lifetime=command->lifetime;	
	throttle_command_.header=command->header;    
    
}

void Manager::commandCallback(const atlascar_base::ManagerCommandPtr& command)
{
	command_queue_.push_msg(command);
}

void Manager::plcStatusCallback(const atlascar_base::PlcStatusPtr& status)
{	
	status_.clutch = status->clutch;
	status_.brake = status->brake;
	status_.steering_wheel = status->steering_wheel;
	status_.rpm = status->rpm;
	
	status_.lights_brake=status->lights_brake;
	status_.lights_reverse=status->lights_reverse;
	status_.lights_warning=status->lights_warning;
	
	status_.emergency=status->emergency;
	status_.auto_ignition=status->auto_ignition;
	status_.auto_brake=status->auto_brake;
	status_.auto_direction=status->auto_direction;
	status_.auto_clutch=status->auto_clutch;
}

void Manager::driverStatusCallback(const human_driver_monitor::HumanDriverMonitorStatusPtr& status)
{
	status_.lights_high=status->lights_high;
	status_.lights_medium=status->lights_medium;
	status_.lights_left=status->lights_left;
	status_.lights_right=status->lights_right;
	status_.lights_danger=status->lights_danger;

	status_.ignition=status->ignition;

	status_.horn=status->horn;

	status_.throttle_pressure=status->throttle_pressure;
	status_.brake_pressure=status->brake_pressure;
	status_.clutch_pressure=status->clutch_pressure;
}

void Manager::odometerStatusCallback(const odometer::OdometerStatusPtr& status)
{
	status_.velocity=status->velocity;
    status_.count=status->count;
    status_.pulses_sec=status->pulses_sec;
    status_.revolutions_sec=status->revolutions_sec;
}

void Manager::gearboxStatusCallback(const atlascar_base::GearboxStatusPtr& status)
{
	status_.gear=status->gear;
	status_.gearbox_status=status->status;
}

void Manager::throttleStatusCallback(const atlascar_base::ThrottleStatusPtr& status)
{
	status_.throttle=status->throttle;
	status_.throttle_pedal=status->footPedal;
	status_.auto_throttle=status->mode;
}

Manager::CarState Manager::getCurrentState()
{
    double limit = 1.2;
    double zero = 0.001;

    CarState new_state = current_state;
    
    if( abs(status_.velocity) < zero && abs(command_->velocity) < zero)
        new_state = STOPPED;
    
    if( abs(status_.velocity) < limit && abs(command_->velocity) > limit)
        new_state = STOPPED_TO_MOVING;
    
    if( abs(status_.velocity) > limit && abs(command_->velocity) > limit)
        new_state = MOVING;
    
    if( abs(status_.velocity) > zero && abs(command_->velocity) < zero)
        new_state = MOVING_TO_STOPPED;
    
    if(new_state != current_state && new_state == STOPPED_TO_MOVING)
        stm_state = FIRST;
    
    if(new_state != current_state && new_state == MOVING_TO_STOPPED)
        mts_state = FIRST;
    
    current_state = new_state;
    
    return current_state;
    
}

void Manager::movingToStopped()
{
    switch(mts_state)
    {
        //first stage
        case FIRST:            
            throttle_command_.throttle = 0;
			throttle_command_.mode = 1;
            plc_command_.clutch=1;
			plc_command_.auto_clutch=1;
            plc_command_.brake=1;
            plc_command_.auto_brake=1;
            
            break;
    }
}

void Manager::stoppedToMoving()
{    
    switch(stm_state)
    {
        //first stage
        case FIRST:
            throttle_command_.throttle = 0;
			throttle_command_.mode = 1;
            plc_command_.clutch=1;
			plc_command_.auto_clutch=1;
            plc_command_.brake=1;
			plc_command_.auto_brake=1;
            
            if (command_->velocity>0)
            {
                gearbox_command_.gear=1;
				
                //exit stage
                if (status_.gear==1)
                    stm_state = SECOND;
            }            
            
            if (command_->velocity<0)
            {
                gearbox_command_.gear=6;
				
                //exit stage
                if (status_.gear==6)
                    stm_state = SECOND;
            } 
            
            break;            
         
        //second stage
        case SECOND: 
            throttle_command_.throttle = 0.85;
            throttle_command_.mode = 1;
            plc_command_.clutch=0;
            plc_command_.auto_clutch=1;
            plc_command_.brake=0;
            plc_command_.auto_brake = 1; 
			
            break;    
			
		default:
			cout<<"Manager::stoppedToMoving::stm_state::default"<<endl;
    }    
}

void Manager::stateManager(const atlascar_base::ManagerCommandPtr& command)
{
    double increment = 0.01;
	
	current_state = getCurrentState();
    
	switch(current_state)
    {        
        case UNKNOWN:
            break;
            
        case INIT:   
            break;
            
        case STOPPED:  
            //stage
            throttle_command_.throttle = 0;
            throttle_command_.mode = 1;			
			plc_command_.clutch=1;
			plc_command_.auto_clutch=1;			
			plc_command_.brake = 1;
			plc_command_.auto_brake = 1;
            
            if (command_->parking==1)
                gearbox_command_.gear=0;
						
            break;
            
        case STOPPED_TO_MOVING: 
            stoppedToMoving();
			
            break;
            
        case MOVING:     //implementation of autonomous velocity control         
			//stage
            throttle_command_.throttle=0.6;
			throttle_command_.mode = 1;
            plc_command_.clutch=0;
            plc_command_.auto_clutch=1;
            plc_command_.brake=0;
			plc_command_.auto_brake = 1;
            
            break;
            
        case MOVING_TO_STOPPED:
            movingToStopped();
			
            break;
            
        default:
            cout<<"This state was not defined: "<<current_state<<endl;
    };
}

void Manager::loop()
{    
	ros::Rate r(20);//Hz
	
	do
	{
		//Get top layer message, the command message that has the most priority and is still valid
		command_=command_queue_.top_msg();
		
		if(command_->direct_control==1)//Direct command mode
		{
			//Put command message into outgoing messages
			toOutgoingMessages(command_);
			
			//Publish all messages
			command_plc_pub_.publish(plc_command_);
			command_gearbox_pub_.publish(gearbox_command_);
			command_throttle_pub_.publish(throttle_command_);
		}else if(command_->direct_control==2)//High command mode
		{
			
			
			stateManager(command_);
            plc_command_.auto_direction=1;
			plc_command_.steering_wheel=command_->steering_wheel;
			//Publish all messages
			
			plc_command_.priority=command_->priority;
			plc_command_.lifetime=command_->lifetime;
			
			command_plc_pub_.publish(plc_command_);
			
			gearbox_command_.priority=command_->priority;
			gearbox_command_.lifetime=command_->lifetime;
			
			command_gearbox_pub_.publish(gearbox_command_);
			
			throttle_command_.priority=command_->priority;
			throttle_command_.lifetime=command_->lifetime;
			
			command_throttle_pub_.publish(throttle_command_);
		}
		
		//Publish the status message
		status_.header.stamp = ros::Time::now();
		status_.header.frame_id = "";
		status_pub_.publish(status_);
		
		//Spin ros and sleep the desired amount
		ros::spinOnce();
		r.sleep();
	}while(ros::ok());
}

}
