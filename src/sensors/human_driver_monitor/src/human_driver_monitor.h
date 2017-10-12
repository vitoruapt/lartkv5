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
#ifndef _HUMAN_DRIVER_MONITOR_H_
#define _HUMAN_DRIVER_MONITOR_H_

/**
\file
\brief HumanDriverMonitor class declaration
*/

#include <ros/ros.h>
#include <iostream>

#include <human_driver_monitor/HumanDriverMonitorStatus.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <boost/thread/thread.hpp>

#include <tcp_client/AsyncClient.h>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

using namespace std;

namespace human_driver_monitor
{

/**
\brief Human driver monitor class

This class handles all communication with the human driver monitor Arduino.
This system monitors the human driver interaction with some of the vehicle actuators, namely: lights and pedals.
*/
class HumanDriverMonitor
{
	public:
		/**
		\name Mandatory common functions
		These functions are mandatory to all low level classes in the atlacar_base namespace.
		All these must operate in a similar fashion across class.
		The constructor must only initialize variable values, all major initializations must be done in the init() function, setupMessaging() subscribes and advertises command messages and status (in that order).
		The loop() function will block the program in a constant loop relaying information to the hardware.
		
		The loop() function can operate in two fashions: setting a spin rate and calling ros::spinOnce() or calling directly ros::spin() if the command callback has been setup properly.
		\warning These functions are MANDATORY
		@{*/
		/**
		\brief Class constructor
		
		Mandatory. Should only be used for variable initialization.
		
		The constructor is used to set the server ip and port, initializing the communications service and object and the ros node handler.
		It also initializes the frequency diagnostics tool with the allowed maximum and minimum frequency.
		*/
		HumanDriverMonitor(const ros::NodeHandle& nh,std::string ip,std::string port):
		server_ip_(ip),
		server_port_(port),
		nh_(nh),
		status_freq_("HumanDriverMonitor",updater_,diagnostic_updater::FrequencyStatusParam(&status_max_frequency_,&status_min_frequency_, 0.1, 10)),//The last two parameters correspond to the tolerance and window size
		status_max_frequency_(50.0),//The frequency values appear in hz
		status_min_frequency_(70.0),
		comm_(io_service_,server_ip_,server_port_)
		{}
		
		/**
		\brief Class destructor
		*/
		~HumanDriverMonitor()
		{}
		
		/**
		\brief Initialize the class
		
		Mandatory. To specific initialization tasks, anything that cannot be preformed in the constructor
		should be done here.
		
		This function initializes the diagnostics tool and sets the communication handlers for received data and connection established.
		*/
		void init()
		{
			//Set diagnostics
			updater_.setHardwareID("human driver monitor");
			updater_.add("HumanDriverMonitor",this,&human_driver_monitor::HumanDriverMonitor::diagnostics);
		
			//Setup the new data handler
			comm_.readHandler.connect(boost::bind(&HumanDriverMonitor::newData,this,_1));
		
			//Setup the on successful connection handler
			comm_.connectHandler.connect(boost::bind(&HumanDriverMonitor::connectHandler,this));
		}
		
		/**
		\brief Start ros message subscribing and advertising
		
		Mandatory. Subscribe command messages and advertise status messages.
		
		This function only advertises the status messages.
		*/
		void setupMessaging()
		{
			status_pub_ = nh_.advertise<human_driver_monitor::HumanDriverMonitorStatus>("status", 1);
		}
		
		/**
		\brief Start main control loop
		
		Mandatory. Do the main loop of the program, call ros spin or spinOnce as needed, should only quit on ros exit command.
		
		This function launches the io service in a separate thread and then starts to do spin ros.
		The while cycle is used to update the diagnostics tool only.
		Once the ros::ok() returns false the io service is stopped the thread closed and the loop quits.
		*/
		void loop()
		{
			//Run io service on a different tread
			boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_service_));
			
			ros::Rate r(50);//Hz
			
			do
			{
				//Update the diagnostics tool
				updater_.update();
				//Spin ros and sleep the desired amount
				ros::spinOnce();
				r.sleep();
			}while(ros::ok());
			
			//Stop the io service running in a separate thread
			io_service_.stop();
			//Join the thread
			thread.join();
		}

		/**
		@}
		*/
		
	protected:
		
		/**
		\brief This function will be called (asynchronously) on a successful connection
		
		This function sends the start command to the Arduino, this order will signal the Arduino to start sending data.
		*/
		void connectHandler(void)
		{
			try
			{
				comm_.write("start");
			}catch(std::exception& e)
			{
				std::cout << "Exception: " << e.what() << "\n";
			}
		}

		/**
		\brief This function will be called (asynchronously) upon arrival of new data
		\param data incoming data in std::string format
		
		This function handles new data coming from the Arduino. The new data is interpreted and a status message is published.
		*/
		void newData(string data)
		{
			//Erase the first 02 and the last 03 (stx and enx)
			data.erase(0,1);
			data.erase(data.end()-2,data.end());
			
			//Get info from message
			
			if(data[1]=='1')
				status_.lights_high=1;
			else
				status_.lights_high=0;
			
			if(data[5]=='1')
				status_.lights_medium=1;
			else
				status_.lights_medium=0;
			
			if(data[2]=='1')
				status_.ignition=1;
			else
				status_.ignition=0;
			
			if(data[6]=='1')
				status_.horn=1;
			else
				status_.horn=0;
			
			if(data[3]=='1')
				status_.lights_left=1;
			else
				status_.lights_left=0;
			
			if(data[4]=='1')
				status_.lights_right=1;
			else
				status_.lights_right=0;
			
			if(status_.lights_left && status_.lights_right)
				status_.lights_danger=1;
			else
				status_.lights_danger=0;
			
			data.erase(0,7);
			
			//use boost spirit qi to parse the rest of the message
			namespace qi = boost::spirit::qi;
			using qi::omit;
			using qi::int_;
			using qi::char_;
			using qi::ascii::space;
			using qi::_1;
			using boost::phoenix::ref;
			
			qi::phrase_parse(data.begin(),data.end(), 
											omit[char_] >> int_[ref(status_.throttle_pressure) = _1] >>
											omit[char_] >> int_[ref(status_.brake_pressure) = _1] >>
											omit[char_] >> int_[ref(status_.clutch_pressure) = _1]
											,space);
			
			status_.header.stamp = ros::Time::now();
			status_.header.frame_id = "";
			status_pub_.publish(status_);
			status_freq_.tick();
		}

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
				stat.add("Connecting to",server_ip_);
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

		///Ip of the Arduino server
		std::string server_ip_;
		///Port of the Arduino server
		std::string server_port_;
				
		///Ros node handler
		ros::NodeHandle nh_;
		
		///Ros status publisher
		ros::Publisher status_pub_;
		
		///Status message
		human_driver_monitor::HumanDriverMonitorStatus status_;
		
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
};

}

#endif
