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
#ifndef _PLC_H_
#define _PLC_H_

/**
\file
\brief Plc class declaration
*/

#include <ros/ros.h>
#include <iostream>

// #include <tcp_client/class_tcp.h>
#include <tcp_client/AsyncClient.h>

#include <atlascar_base/PlcStatus.h>
#include <atlascar_base/PlcCommand.h>

#include <topic_priority/topic_priority.h>

#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>


#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

using namespace std;

namespace atlascar_base
{

class SimpleCalibration 
{
	public:
		
		double maximum_value_;
		double minimum_value_;
		
		double maximum_required_;
		double minimum_required_;
	
		/**
		 * \brief Linear mapping function
		 * \param value input value
		 * \return the remapped value
		 * 
		 * This function maps a value to a different scale.
		 */
		double remap(double value)
		{
			assert(minimum_value_!=maximum_value_);
			assert(minimum_required_!=maximum_required_);
			
			if(minimum_value_>maximum_value_)
				return remapInverted(value);
			
			if(value<minimum_value_)
				value=minimum_value_;
			
			if(value>maximum_value_)
				value=maximum_value_;
			
			double m = (maximum_required_-minimum_required_)/(maximum_value_-minimum_value_);
			double b = maximum_required_ - m*maximum_value_;
			
			return value*m+b;
		}
		
		
		double remapInverted(double value)
		{
			assert(minimum_value_!=maximum_value_);
			assert(minimum_required_!=maximum_required_);
			
			if(value>minimum_value_)
				value=minimum_value_;
			
			if(value<maximum_value_)
				value=maximum_value_;
			
			double m = (maximum_required_-minimum_required_)/(maximum_value_-minimum_value_);
			double b = maximum_required_ - m*maximum_value_;
			
			return value*m+b;
		}
		
	private:
		
};
	
/**
\brief Plc communication and control class

This class implements the Ethernet and Plc communication protocol in order to establish a
communication with it. The communication is maintained and a status message is continuously
published.
*/
class Plc
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
		or calling directly ros::spin() if the command callback has been setup properly. In this class (Plc)
		it works by calling the ros::spinOnce() at a predefined rate.
		\warning These functions are MANDATORY
		@{*/
		
		/**
		\brief Class constructor
		
		Initialize variables, do not call any function
		*/
		Plc(const ros::NodeHandle& nh,std::string ip, std::string port);
		
		/**
		\brief Class destructor
		
		Do nothing.
		*/
		~Plc();
		
		/**
		\brief Initialize the class
		
		Add the safety message to the command queue.
		*/
		void init();
		
		/**
		\brief Start ros message subscribing and advertising
		
		Subscribe command messages and advertise status messages.
		*/
		void setupMessaging();
		
		/**
		\brief Start main control loop
		
		Loop the module sending command messages to the plc, this function will only exit
		of the ros exit command. It currently loops at about 15Hz which is the plc maximum
		communication speed.
		*/
		void loop();
		
		/**
		@}
		*/
		
	private:
		
		/**
		\brief Diagnostics function handler
		\param stat diagnostics information storage
		
		This function preforms diagnostics on the communication with the Arduino and reports back to the diagnostics tool.
		*/
		void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
		{
			if(!comm_.isConnected())
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No connection to hardware.");
				stat.add("Status",comm_.error_.message());
				stat.add("Trying to connect to",server_ip_);
				stat.add("Port number",server_port_);
			}else
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No problems.");
				stat.add("Connected with",server_ip_);
				stat.add("Port number",server_port_);
				
				boost::format fmt("%.2f");
				fmt % (ros::Time::now()-status_.header.stamp).toSec();
				
				stat.add("Last message sent", fmt.str()+" seconds ago" );
			}
		}
		
		/**
		\brief This function will be called (asynchronously) upon arrival of new data
		\param data incoming data in std::string format

		This function handles new data coming from the Arduino. The new data is interpreted and a status message is published.
		*/
		void newData(string data);
		
		/**
		\brief Send a global command down to the plc
		
		This function prepares the command message to send, sends it and receives the plc answer.
		\return error code, 0 if no error <0 on error
		*/
		int sendCommand(void);
		
		/**
		\brief Send a message
		
		Format the string to send, and send it.
		\return error code, 0 if no error <0 on error
		*/
		int sendMessage(char*message_string);

		/**
		\brief Command message handler
		
		Receive  command messages and put them in the queue.
		\param command received command message
		*/
		void commandCallback(const atlascar_base::PlcCommandPtr& command);
		
		/**
		\name Class variables
		The class variables can be both public or private. They should preferably be private whenever
		possible.
		@{*/
		
		///Calibration of the steering wheel from the plc
		SimpleCalibration steering_wheel_calibration_;
		///Calibration of the steering wheel to the plc
		SimpleCalibration steering_wheel_to_plc_calibration_;
		
		///Calibration of the brake pedal
		SimpleCalibration brake_calibration_;
		///Calibration of the clutch pedal
		SimpleCalibration clutch_calibration_;
		
		///Ip of the Plc server
		std::string server_ip_;
		///Port of the Plc server
		std::string server_port_;
		
		///Ros node handler
		ros::NodeHandle nh_;
		///Ros command subscriber
		ros::Subscriber command_sub_;
		///Ros status publisher
		ros::Publisher status_pub_;
		///Command queue holding class
		TopicQueuePriority<atlascar_base::PlcCommandPtr> command_queue_;		
		///Current connection status
		int connection_status_;
		///Verbose mode
		bool verbose_;
		///Message received
		char received_message_[1024];
		///Command message pointer
		atlascar_base::PlcCommandPtr command_;
		///Safety command message pointer
		atlascar_base::PlcCommandPtr safety_command_;
		///Status message
		atlascar_base::PlcStatus status_;
		
		///Diagnostics class
		diagnostic_updater::Updater updater_;
		///Frequency diagnostics tool
		diagnostic_updater::HeaderlessTopicDiagnostic status_freq_;
		///Maximum admissible frequency
		double status_max_frequency_;
		///Minimum admissible frequency
		double status_min_frequency_;
		
		///Input/Output communication service
		boost::asio::io_service io_service_;
		///Asynchronous tcp/ip communication object
		AsyncClient comm_;
		/**
		@}
		*/
};

}

#endif
