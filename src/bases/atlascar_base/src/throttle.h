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
#ifndef _THROTTLE_H_
#define _THROTTLE_H_

/**
\file
\brief Throttle class declaration
*/

#include <ros/ros.h>
#include <iostream>
#include <boost/lexical_cast.hpp>

#include <tcp_client/AsyncClient.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <atlascar_base/ThrottleStatus.h>
#include <atlascar_base/ThrottleCommand.h>

#include <topic_priority/topic_priority.h>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

using namespace std;

namespace atlascar_base
{

/**
\brief Throttle monitor and control class

This class handles all communication with the atlascar throttle controller.
This system controls the vehicle throttle.
*/
class Throttle
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
		or calling directly ros::spin() if the command callback has been setup properly. In this class (Throttle)
		it works by calling the ros::spinOnce() at a predefined rate.
		\warning These functions are MANDATORY
		@{*/
		/**
		\brief Class constructor
		
		Mandatory. Should only be used for variable initialization.
		
		The constructor is used to set the server ip and port, initializing the communications service and object and the ros node handler.
		It also initializes the frequency diagnostics tool with the allowed maximum and minimum frequency.
		*/
		Throttle(const ros::NodeHandle& nh,std::string ip,std::string port):
		nh_(nh),
		current_mode_(MANUAL),
		server_ip_(ip),
		server_port_(port),	status_freq_("Throttle",updater_,diagnostic_updater::FrequencyStatusParam(&status_min_frequency_,&status_max_frequency_, 0.01, 10)),
		status_max_frequency_(51.0),
		status_min_frequency_(49.0),
		comm_(io_service_,server_ip_,server_port_)
		{}
		
		/**
		\brief Class destructor
		*/
		~Throttle()
		{}
		
		/**
		\brief Initialize the class
		
		Mandatory. To specific initialization tasks, anything that cannot be preformed in the constructor
		should be done here.
		
		This function initializes the diagnostics tool and sets the communication handlers for received data and connection established.
		*/
		void init()
		{
			safety_command_.reset(new atlascar_base::ThrottleCommand);
			
			//Set the safety message default values, this messages will never be removed from the list
			safety_command_->lifetime=INFINITY;
			safety_command_->priority=0;
			safety_command_->throttle=0;
			safety_command_->mode=MANUAL;
			
			//Push the safety message into the queue
			command_queue_.push_msg(safety_command_);
			
			//Set diagnostics
			updater_.setHardwareID("throttle");
			updater_.add("Throttle",this,&atlascar_base::Throttle::diagnostics);
			
			//Setup the new data handler
			comm_.readHandler.connect(boost::bind(&Throttle::newData,this,_1));
			
			//Setup the on successful connection handler
			comm_.connectHandler.connect(boost::bind(&Throttle::connectHandler,this));
		}
		
		/**
		\brief Start ros message subscribing and advertising
		
		Mandatory. Subscribe command messages and advertise status messages.
		
		This function advertises the status messages and subscribes command messages.
		*/
		void setupMessaging()
		{
			command_sub_ = nh_.subscribe("command", 1, &Throttle::commandCallback, this);
			status_pub_ = nh_.advertise<atlascar_base::ThrottleStatus>("status", 1);
		}
		
		/**
		\brief Start main control loop
		
		Mandatory. Do the main loop of the program, call ros spin or spinOnce as needed, should only quit on ros exit command.
		
		This function launches the io service in a separate thread and then starts to do spin ros.
		The while cycle is used to update the diagnostics tool and send commands to the controller.
		Once the ros::ok() returns false the io service is stopped the thread closed and the loop quits.
		*/
		void loop()
		{
			//Run io service on a different tread
			boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_service_));
			
			ros::Rate r(50);//Hz
			
			do
			{
				updater_.update();
				
				//Get top layer message, the command message that has the most priority and is still valid
				command_=command_queue_.top_msg();
								
				//If the demanded mode is different from the current mode, set the mode
				if(status_.mode!=(int)command_->mode)
				{
					boost::format fmt("set mode %d\n");
					fmt % command_->mode;

					try
					{
						comm_.write(fmt.str());
					}catch(std::exception& e)
					{
						std::cout<<"Cannot set mode: "<<e.what()<<std::endl;
					}
				}
				
				//If we are in auto mode send throttle
				if(status_.mode==AUTO)
				{
					boost::format fmt("set throttle %lf\n");
					fmt % command_->throttle;
					
					try
					{
						comm_.write(fmt.str());
					}catch(std::exception& e)
					{
						std::cout<<"Cannot set throttle: "<<e.what()<<std::endl;
					}
				}
				
				if( comm_.isConnected() &&  (ros::Time::now()-status_.header.stamp).toSec() > 1.0)
				{
					comm_.reconnect();
				}
				
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
	private:

		enum {MANUAL=0,AUTO=1};
		
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
				
				boost::format fmt("%.4f");
				fmt % (ros::Time::now()-status_.header.stamp).toSec();
			
				stat.add("Last message received", fmt.str()+" seconds ago" );
			}
		}
		
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
			
			status_.header.stamp=ros::Time::now();
		}
		
		/**
		\brief This function handles newly received commands
		\param command received command message
		This function pushes the message to the command queue.
		*/
		void commandCallback(const atlascar_base::ThrottleCommandPtr& command)
		{
			command_queue_.push_msg(command);
		}
		
		/**
		\brief This function will be called (asynchronously) upon arrival of new data
		\param data incoming data in std::string format
		
		This function handles new data coming from the Arduino. The new data is interpreted and a status message is published.
		This function uses boost::qi parser to interpret the message, this parser is quite complicated but very expansible, see the documentation for a tutorial on its use (it's pretty funny).
		*/
		void newData(string data)
		{
			namespace qi = boost::spirit::qi;
			using qi::double_;
			using qi::int_;
			using qi::ascii::space;
			using boost::spirit::ascii::string;
			using qi::_1;
			using boost::phoenix::ref;
			
			//Parse the received data, the 3 strings must match exactly
			qi::phrase_parse(data.begin(),data.end(), 
											string("mode") >> int_[ref(status_.mode) = _1] >>
											string("throttle") >> double_[ref(status_.throttle) = _1] >>
											string("pedal") >> double_[ref(status_.footPedal) = _1]
											,space);
			
			status_.header.stamp = ros::Time::now();
			status_.header.frame_id = "";
			status_pub_.publish(status_);
			status_freq_.tick();
		}
		
		///Ros node handler
		ros::NodeHandle nh_;
		///Ros command subscriber
		ros::Subscriber command_sub_;
		///Ros status publisher
		ros::Publisher status_pub_;
		///Command queue holding class
		TopicQueuePriority<atlascar_base::ThrottleCommandPtr> command_queue_;
		///Command message pointer
		atlascar_base::ThrottleCommandPtr command_;
		///Safety command message pointer
		atlascar_base::ThrottleCommandPtr safety_command_;
		///Status message
		atlascar_base::ThrottleStatus status_;
		///Current working mode
		int current_mode_;
		
		///Throttle server ip
		std::string server_ip_;
		///Throttle server port
		std::string server_port_;
			
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
