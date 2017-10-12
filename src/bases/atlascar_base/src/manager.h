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
#ifndef _MANAGER_H_
#define _MANAGER_H_

/**
\file
\brief Manager class declaration
*/

#include <ros/ros.h>
#include <iostream>

#include <atlascar_base/ManagerStatus.h>
#include <atlascar_base/ManagerCommand.h>

#include <atlascar_base/GearboxStatus.h>
#include <atlascar_base/GearboxCommand.h>

#include <human_driver_monitor/HumanDriverMonitorStatus.h>
#include <odometer/OdometerStatus.h>

#include <atlascar_base/ThrottleStatus.h>
#include <atlascar_base/ThrottleCommand.h>

#include <atlascar_base/PlcStatus.h>
#include <atlascar_base/PlcCommand.h>

#include <topic_priority/topic_priority.h>

using namespace std;

namespace atlascar_base
{

/**
\brief Global management and control class

This class is responsible for the synchronization, calibration and contextual control for the 
atlascar vehicle.
The class communicates locally with the \ref low_level "low level modules" via 
command and status messages.
The received status messages must be interpreted and calibrated while being integrated into 
global asynchronous status publisher. The global atlascar_base::ManagerStatus in periodically 
published independently of the frequency of incoming partial status messages.
The manager receives global control messages atlascar_base::ManagerCommand.
*/
class Manager
{
	public:
		/**
		\name Mandatory common functions
		These functions are mandatory to all low level classes in the atlacar_base namespace.
		All these must operate in a similar fashion across class.
		The constructor must only initialize variable values, all major initializations must
		be done in the init() function, setupMessaging() subscribes and advertises command messages
		and status (in that order). The loop() function will block the program in a constant loop
		relaying information to the hardware.
		
		The loop() function can operate in two fashions: setting a spin rate and calling ros::spinOnce()
		or calling directly ros::spin() if the command callback has been setup properly. In this class (Manager)
		it works by calling the ros::spinOnce() at a predefined rate.
		\warning These functions are MANDATORY
		@{*/
		
		/**
		\brief Class constructor
		
		Initialize variables, do not call any function
		*/
		Manager(const ros::NodeHandle& nh);
		
		/**
		\brief Class destructor
		
		Do nothing.
		*/
		~Manager();
		
		/**
		\brief Initialize the class
		
		Add the safety message to the command queue.
		*/
		void init();
		
		/**
		\brief Start ros message subscribing and advertising
		
		Subscribe local command messages and advertise local status messages.
		Subscribe to all low level status messages, and advertise all low level command messages.
		*/
		void setupMessaging();
		
		/**
		\brief Start main control loop
		
		Start ros spin and main state machine.
		*/
		void loop();
		/**
		@}
		*/
        
        /**
        \brief Car state machine status
        
        Define all states of macine 
        */      
        typedef enum 
        {
            UNKNOWN,
            INIT,
            
            STOPPED,
            MOVING,
            STOPPED_TO_MOVING,
            MOVING_TO_STOPPED            
        } CarState;
        
        /**
        \brief Car stages
        
        Define all stages
        */ 
        typedef enum 
        {
            FIRST,
            SECOND,
            THIRD
        } sequentialState;
        
        
	private:
		
		/**
		\brief Main command callback
		
		Receive command messages from outside the atlascar_base. 
		*/
		void commandCallback(const atlascar_base::ManagerCommandPtr& command);
		
        /**
        \brief Check Current State
        
        Analyse the velocity and choose the current state 
        */
        CarState getCurrentState();
        
		/**
		\brief Plc status callback
		
		Receive status messages from the Plc low level module
		*/
		void plcStatusCallback(const atlascar_base::PlcStatusPtr& status);
		
		/**
		\brief Driver status callback
		
		Receive status messages from the Driver low level module
		*/
		void driverStatusCallback(const human_driver_monitor::HumanDriverMonitorStatusPtr& status);
		
		/**
		\brief Velocity status callback
		
		Receive status messages from the Velocity low level module
		*/
		void odometerStatusCallback(const odometer::OdometerStatusPtr& status);
		
		/**
		\brief Gearbox status callback
		
		Receive status messages from the Gearbox low level module
		*/
		void gearboxStatusCallback(const atlascar_base::GearboxStatusPtr& status);
		
		/**
		\brief Throttle status callback
		
		Receive status messages from the Throttle low level module
		*/
		void throttleStatusCallback(const atlascar_base::ThrottleStatusPtr& status);	
	
        /**
        \brief Messages
        
        Put command message into outgoing messages
        */
		void toOutgoingMessages(const atlascar_base::ManagerCommandPtr& command);
		
        /**
        \brief State Manager
        
        Analyzes the state of state machine
        */
		void stateManager(const atlascar_base::ManagerCommandPtr& command);
		
        /**
        \brief Stopped to Moving
        
        Redefine values of variables to be send
        */
        void stoppedToMoving();
        
        /**
        \brief Moving to Stopped
        
        Redefine values of variables to be send
        */
        void movingToStopped();
        
        
        ///Stopped to moving state
        sequentialState stm_state;
        ///Moving to stopped state 
        sequentialState mts_state; 
        ///Current state of state machine
        CarState current_state; 
        
        uint target_gear_;
		///Ros node handler
		ros::NodeHandle nh_;
		///Ros command subscriber
		ros::Subscriber command_sub_;
		///Plc status subscriber
		ros::Subscriber status_plc_sub_;
		///Driver status subscriber
		ros::Subscriber status_driver_sub_;
		///Velocity status subscriber
		ros::Subscriber status_odometer_sub_;
		///Gearbox status subscriber
		ros::Subscriber status_gearbox_sub_;
		///Throttle status subscriber
		ros::Subscriber status_throttle_sub_;
		///Ros status publisher
		ros::Publisher status_pub_;
		///Plc command publisher
		ros::Publisher command_plc_pub_;
		///Gearbox command publisher
		ros::Publisher command_gearbox_pub_;
		///Throttle command publisher
		ros::Publisher command_throttle_pub_;
		///Command queue holding class
		TopicQueuePriority<atlascar_base::ManagerCommandPtr> command_queue_;		
		///Verbose mode
		bool verbose_;
		///Command message pointer
		atlascar_base::ManagerCommandPtr command_;
		///Safety command message pointer
		atlascar_base::ManagerCommandPtr safety_command_;
		///Status message
		atlascar_base::ManagerStatus status_;
		///Gearbox command message
		atlascar_base::GearboxCommand gearbox_command_;
		///Gearbox status message
		atlascar_base::GearboxStatus gearbox_status_;
		///Plc command message
		atlascar_base::PlcCommand plc_command_;
		///Plc status message
		atlascar_base::PlcStatus plc_status_;
		///Driver status message
		human_driver_monitor::HumanDriverMonitorStatus driver_status_;
		///Throttle command message
		atlascar_base::ThrottleCommand throttle_command_;
		///Throttle status message
		atlascar_base::ThrottleStatus throttle_status_;
		///Velocity status message
		odometer::OdometerStatus odometer_status_;
};

}

#endif
