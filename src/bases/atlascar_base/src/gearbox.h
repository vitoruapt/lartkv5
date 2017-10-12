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
#ifndef _GEARBOX_H_
#define _GEARBOX_H_

/**
\file
\brief Gearbox class declaration
*/

#include <ros/ros.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <tcp_client/class_tcp.h>

#include <atlascar_base/GearboxStatus.h>
#include <atlascar_base/GearboxCommand.h>

#include <topic_priority/topic_priority.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace atlascar_base
{

/**
\brief This class is not fully documented!

\warning Missing documentation!!

The documentation of this class is the responsibility of masters student Pedro Pinheiro (assigned
by Jorge Almeida)
*/
class Gearbox
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
		or calling directly ros::spin() if the command callback has been setup properly. In this class (Gearbox)
		it works by calling the ros::spinOnce() at a predefined rate.
		\warning These functions are MANDATORY
		@{*/
		/**
		\brief Class constructor
		
		Mandatory. Should only be used for variable initialization.
		*/
		Gearbox(const ros::NodeHandle& nh);
		
		/**
		\brief Class destructor
		*/
		~Gearbox();
		
		/**
		\brief Initialize the class
		
		Mandatory. To specific initialization tasks, anything that cannot be preformed in the constructor
		should be done here.
		*/
		void init();
		
		/**
		\brief Start ros message subscribing and advertising
		
		Mandatory. Subscribe command messages and advertise status messages.
		*/
		void setupMessaging();
		
		/**
		\brief Start main control loop
		
		Mandatory. Do the main loop of the program, call ros spin or spinOnce as needed, should only quit on ros exit command.
		*/
		void loop();
		/**
		@}
		*/
	private:
		
		void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
		{
			if(connection_status_==OFFLINE)
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No connection to hardware.");
				stat.add("Status","Check ethernet connection!");
				stat.add("Trying to connect to",ip_);
			}else
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No problems.");
		}
		
		/**
		\brief Keep connection alive
		
		Maintains the connection with the hardware active, reconnects if needed.
		\return 0 if connection active or reconnection successful, -1 on reconnection fail
		*/
		int maintainConnection();
		
		/**
		\brief Command message handler
		
		Receive  command messages and put them in the queue.
		\param command received command message
		*/
		void commandCallback(const atlascar_base::GearboxCommandPtr& command);
		
		int receiveMessage(string& message);
		
		int interpreterMessage(string& message);
		
		///Online and offline states
		enum {OFFLINE=0, ONLINE};
		
		///Ros node handler
		ros::NodeHandle nh_;
		///Ros command subscriber
		ros::Subscriber command_sub_;
		///Ros status publisher
		ros::Publisher status_pub_;
		///TCP client communication class pointer
		tcp_client* comm_;
		///Command queue holding class
		TopicQueuePriority<atlascar_base::GearboxCommandPtr> command_queue_;
		///Current connection status
		int connection_status_;
		///Verbose mode
		bool verbose_;
		///Command message pointer
		atlascar_base::GearboxCommandPtr command_;
		///Safety command message pointer
		atlascar_base::GearboxCommandPtr safety_command_;
		///Status message
		atlascar_base::GearboxStatus status_;
		///
		std::string ip_;
		///
		int port_;
		
		//Create the diagnostics class
		diagnostic_updater::Updater updater_;
		diagnostic_updater::HeaderlessTopicDiagnostic status_freq_;
		double status_max_frequency_;
		double status_min_frequency_;
};

}

#endif
