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
\brief Plc class implementation
*/

#include "plc.h"

namespace atlascar_base
{
	
Plc::Plc(const ros::NodeHandle& nh,std::string ip, std::string port)
:
server_ip_(ip),
server_port_(port),
nh_(nh),
status_freq_("Plc",updater_,diagnostic_updater::FrequencyStatusParam(&status_max_frequency_,&status_min_frequency_, 0.1, 10)),
status_max_frequency_(10.0),
status_min_frequency_(20.0),
comm_(io_service_,server_ip_,server_port_,std::string("\x03"))
{
	verbose_=0;
}

Plc::~Plc()
{}

void Plc::init()
{
	safety_command_.reset(new atlascar_base::PlcCommand);
	
	//Set the safety message default values, this messages will never be removed from the list
	safety_command_->lifetime=INFINITY;
	safety_command_->priority=0;
	safety_command_->brake=0; //This will make the pedal go down
	safety_command_->clutch=0; //This will make the pedal go down
	safety_command_->steering_wheel=0;//center steering wheel
	
	//Push the safety message into the queue
	command_queue_.push_msg(safety_command_);
	
	//Set diagnostics
	updater_.setHardwareID("plc");
	updater_.add("Plc",this,&atlascar_base::Plc::diagnostics);
	
	//Setup the new data handler
	comm_.readHandler.connect(boost::bind(&Plc::newData,this,_1));
	
	//Get the atlascar parameters
	nh_.param("steering_wheel_from_plc_min",steering_wheel_calibration_.minimum_value_,-1.);
	nh_.param("steering_wheel_from_plc_max",steering_wheel_calibration_.maximum_value_,-1.);
	
	nh_.param("steering_wheel_from_plc_rad_min",steering_wheel_calibration_.minimum_required_,-1.);
	nh_.param("steering_wheel_from_plc_rad_max",steering_wheel_calibration_.maximum_required_,-1.);
	
	nh_.param("steering_wheel_to_plc_min",steering_wheel_to_plc_calibration_.minimum_required_,-1.);
	nh_.param("steering_wheel_to_plc_max",steering_wheel_to_plc_calibration_.maximum_required_,-1.);
	
	nh_.param("steering_wheel_to_plc_rad_min",steering_wheel_to_plc_calibration_.minimum_value_,-1.);
	nh_.param("steering_wheel_to_plc_rad_max",steering_wheel_to_plc_calibration_.maximum_value_,-1.);
	
	nh_.param("brake_plc_min",brake_calibration_.minimum_value_,-1.);
	nh_.param("brake_plc_max",brake_calibration_.maximum_value_,-1.);
	
	brake_calibration_.minimum_required_=0;
	brake_calibration_.maximum_required_=1;
		
	nh_.param("clutch_plc_min",clutch_calibration_.minimum_value_,-1.);
	nh_.param("clutch_plc_max",clutch_calibration_.maximum_value_,-1.);
	
	clutch_calibration_.minimum_required_=0;
	clutch_calibration_.maximum_required_=1;
}

void Plc::setupMessaging()
{
	command_sub_ = nh_.subscribe("command", 1, &Plc::commandCallback, this);
	status_pub_ = nh_.advertise<atlascar_base::PlcStatus>("status", 1);
}

void Plc::loop()
{
	//Run io service on a different tread
	boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_service_));

	ros::Rate r(15);//Hz

	do
	{
		updater_.update();
		
		//Get top layer message, the command message that has the most priority and is still valid
		command_=command_queue_.top_msg();
		
		if(!command_)
		{
			cout<<"Error!! Invalid command in queue, safety message removed by user!!"<<endl;
		}else
		{
			try
			{
				//Send the command message to the PLC
				sendCommand();
			}catch(std::exception& e)
			{
				std::cout << "Exception: " << e.what() << "\n";
			}
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

void Plc::newData(string data)
{
	std::size_t found=data.find("\x02");
	data.erase(0,found+1);
	data.erase(data.end()-1,data.end());
	
	double throttle;
	double steering_wheel;
	double brake;
	double clutch;
	double rpm;
	double speed;
	int handbrake;
	int emergency;
	int gear;
	int error;
	
	sscanf(data.c_str(),"%*s %lf %*s %lf %*s %lf %*s %lf %*s %d %*s %d %*s %lf %*s %lf %*s %d %*s %d",&throttle,&steering_wheel,&brake,&clutch,&handbrake,&gear,&speed,&rpm,&emergency,&error);
	
	status_.clutch=clutch_calibration_.remap(clutch);
	status_.brake=brake_calibration_.remap(brake);
	status_.steering_wheel=steering_wheel_calibration_.remap(steering_wheel);
	
	status_.rpm=rpm;
	status_.ignition=command_->ignition;
	status_.emergency=emergency;
	
	/*Get the rest of the stuff from the outgoing message*/
	status_.lights_high=command_->lights_high;
	status_.lights_medium=command_->lights_medium;
	status_.lights_minimum=command_->lights_minimum;
	status_.lights_left=command_->lights_left;
	status_.lights_right=command_->lights_right;
	status_.lights_brake=command_->lights_brake;
	status_.lights_reverse=command_->lights_reverse;
	status_.lights_warning=command_->lights_warning;
	
	status_.auto_ignition=command_->auto_ignition;
	status_.auto_brake=command_->auto_brake;
	status_.auto_direction=command_->auto_direction;
	status_.auto_clutch=command_->auto_clutch;

	status_.header.stamp = ros::Time::now();
	status_.header.frame_id = "";
	status_pub_.publish(status_);
	status_freq_.tick();
}

int Plc::sendCommand(void)
{
	char message_string[2000];
	char text[2000];
	
	double steering_wheel = steering_wheel_to_plc_calibration_.remap(command_->steering_wheel);
	
	sprintf(message_string,"STTP %.4E ",0.0);

	sprintf(text,"SSWP %.4E ",steering_wheel);
	strcat(message_string,text);
	
	sprintf(text,"SBKP %.4E ",command_->brake);
	strcat(message_string,text);
	
	sprintf(text,"SCLP %.4E ",command_->clutch);
	strcat(message_string,text);
	
	sprintf(text,"SHBP %10d ",0);
	strcat(message_string,text);
	
	sprintf(text,"SGER %10d ",0);
	strcat(message_string,text);
	
	sprintf(text,"SIGN %10d ",command_->ignition);
	strcat(message_string,text);
	
	sprintf(text,"SVHS %.4E ",0.0);
	strcat(message_string,text);
	
	sprintf(text,"SVBS %10d ",1);
	strcat(message_string,text);
	
	sprintf(text,"SFAL %10d ",command_->lights_warning);
	strcat(message_string,text);
	
	sprintf(text,"SHIL %10d ",command_->lights_high);
	strcat(message_string,text);
	
	sprintf(text,"SHEL %10d ",command_->lights_medium);
	strcat(message_string,text);
	
	sprintf(text,"SRTL %10d ",command_->lights_right);
	strcat(message_string,text);
	
	sprintf(text,"SLTL %10d ",command_->lights_left);
	strcat(message_string,text);
	
	sprintf(text,"STTA %10d ",0);
	strcat(message_string,text);
	
	sprintf(text,"SSWA %10d ",command_->auto_direction);
	strcat(message_string,text);
	
	sprintf(text,"SBKA %10d ",command_->auto_brake);
	strcat(message_string,text);
	
	sprintf(text,"SCLA %10d ",command_->auto_clutch);
	strcat(message_string,text);
	
	sprintf(text,"SHBA %10d ",0);
	strcat(message_string,text);
	
	sprintf(text,"SGEA %10d ",0);
	strcat(message_string,text);
	
	sprintf(text,"EEST %10d ",command_->emergency);
	strcat(message_string,text);
	
	sprintf(text,"SIGA %10d ",command_->auto_ignition);
	strcat(message_string,text);

	sendMessage(message_string);
	
	return 0;
}

int Plc::sendMessage(char*message_string)
{
	char formatted_message[2000];
	
	memset(formatted_message,0,sizeof(formatted_message));
	sprintf(formatted_message,"%c%s%c",2,message_string,3);
	
	std::string msg = std::string(formatted_message);
	if(verbose_)std::cout<<">> "<<msg<<std::endl;
	comm_.write(std::string(formatted_message));
	
	return 0;
}

void Plc::commandCallback(const atlascar_base::PlcCommandPtr& command)
{
	command_queue_.push_msg(command);
}

}
