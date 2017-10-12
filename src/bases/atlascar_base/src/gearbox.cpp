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
\brief Gearbox class implementation
*/

#include "gearbox.h"

namespace atlascar_base
{


Gearbox::Gearbox(const ros::NodeHandle& nh)
:nh_(nh),
// server_ip_gearbox(ip_),
// server_port_gearbox(port_),
// ip_("192.168.1.123"),
ip_("10.0.0.213"),
port_(120),
status_freq_("Gearbox",updater_,diagnostic_updater::FrequencyStatusParam(&status_max_frequency_,&status_min_frequency_, 0.1, 10)),
status_max_frequency_(8.0),
status_min_frequency_(12.0)
{
	comm_=NULL;
}

Gearbox::~Gearbox()
{
}

void Gearbox::init()
{
	safety_command_.reset(new atlascar_base::GearboxCommand);
	
	//Set the safety message default values, this messages will never be removed from the list
	safety_command_->lifetime=INFINITY;
	safety_command_->priority=0;
	safety_command_->gear=0;
	
	//Push the safety message into the queue
	command_queue_.push_msg(safety_command_);
	
	//Set diagnostics
	updater_.setHardwareID("gearbox");
	updater_.add("Gearbox",this,&atlascar_base::Gearbox::diagnostics);
}

void Gearbox::setupMessaging()
{
	command_sub_ = nh_.subscribe("command", 1, &Gearbox::commandCallback, this);
	status_pub_ = nh_.advertise<atlascar_base::GearboxStatus>("status", 1);
}

void Gearbox::loop()
{
	// String to contain the received message:
	string received_message;
	// String to contain the message to send:
	char send[1024];
	
	ros::Rate r(12);//Hz
	
	do
	{
		updater_.update();
		
		//Get top layer message, the command message that has the most priority and is still valid
		command_=command_queue_.top_msg();
		
		//Check the status of the connection and reconnect if needed
		maintainConnection();
		
		//Commandar e ler o estado da caixa
		//a variabel de controlo é command_
		//a variabvle de estado é status_
		
		if (command_->gear<7 && connection_status_==ONLINE)
		{
// 			cout<<"Setting gear: "<<command_->gear<<endl;
			sprintf(send,"sg %d",command_->gear);
			comm_->Send(send,strlen(send)+1);
			comm_->perr("Failed to send");
				
			// If a message is received:
			if(receiveMessage(received_message)==0)
				interpreterMessage(received_message);
		}
		
// 		cout<<"Get current state"<<endl;
		sprintf(send,"gg");
		comm_->Send(send,strlen(send)+1);
		comm_->perr("Failed to send");
		
		if(receiveMessage(received_message)==0)
				interpreterMessage(received_message);
		
		
		//Publish the status message
		status_.header.stamp = ros::Time::now();
		status_.header.frame_id = "";
		status_pub_.publish(status_);
		status_freq_.tick();
		
		//Spin ros and sleep the desired amount
		ros::spinOnce();
		r.sleep();
	}while(ros::ok());
}

void Gearbox::commandCallback(const atlascar_base::GearboxCommandPtr& command)
{
	command_queue_.push_msg(command);
}

int Gearbox::maintainConnection()
{
	if(connection_status_==ONLINE)//Connection is active nothing to do
		return 0;
	
	if(comm_)//if the comm already exists
	{
		//Disconnect Ethernet
		comm_->Disconnect();
		//Delete Ethernet object
		delete comm_;
	}

	comm_=new tcp_client(ip_.c_str(),port_);
	
	errno=0;

	struct timeval timeout = {1,0};//set 1 sec timeout for read and write
	setsockopt(comm_->GetSocket(),SOL_SOCKET,SO_SNDTIMEO,&timeout, sizeof(timeout));
	setsockopt(comm_->GetSocket(),SOL_SOCKET,SO_RCVTIMEO,&timeout, sizeof(timeout));
	
	comm_->Connect();
	if(comm_->err<0)
	{
		std::cout<<"Cannot connect to gearbox: "<<comm_->err_msg<<" "<<strerror(errno)<<std::endl;
		comm_->Disconnect();
		connection_status_=OFFLINE;
		return -1;
	}

	std::cout<<"reconnection sucessfull."<<std::endl;

	connection_status_=ONLINE;

	return 0;
}

int Gearbox::interpreterMessage(string& message)
{	
	char opcode[100];
	char code;
	int current_gear;
	int next_gear;
	
// 	cout<<"Message: >"<<message.c_str()<<"<"<<endl;
	
	sscanf(message.c_str(),"%*c %s",opcode);
	
	if (opcode[0]=='u')
	{
		status_.status="command unknown";
		return -1;
	}
	
	sscanf(message.c_str(),"%*c %*s %c",&code);
	switch (code)
	{
		case 'i':
			status_.status="command invalid";
			break;
		case 'm':
			status_.status="manual mode";
			break;
		case 'c':
			sscanf(message.c_str(),"%*c %*s %*c %d %*c %d",&current_gear,&next_gear);
			status_.gear=current_gear;
			
			status_.status="changing to "+boost::lexical_cast<std::string>(next_gear);
// 			cout<<"GEARBOX: "<<status_.status<<endl;
			
			break;
		case 'a':
			sscanf(message.c_str(),"%*c %*s %*c %d",&current_gear);
			status_.gear=current_gear;
			status_.status="ok";
			
// 			cout<<"GEARBOX: "<<status_.status<<" current gear: "<<current_gear<<endl;
			
			break;
		default:
			status_.status="unrecognised status";
			break;		
	}	
	
	return 0;
}





int Gearbox::receiveMessage(string& message)
{
  char rec_buf[1024];
  char recv_msg[1024];
  memset(recv_msg,0,sizeof(recv_msg));

  while(rec_buf[0]!=2)
  {
    if(comm_->Receive(rec_buf,1)!=0)
    {
      comm_->Disconnect();
      comm_->perr("Cannot read");
    }
    
    if(comm_->err<0)
      return -1;
  }

  int i=0;
  while(rec_buf[0]!=3)
  {
    if(comm_->Receive(rec_buf,1)!=0)
    {
      comm_->Disconnect();
      comm_->perr("Cannot read");
    }
    
    if(comm_->err<0)
      return -1;
    recv_msg[i++]=rec_buf[0];
  }
  
  recv_msg[i-1]='\0';
  message=recv_msg;
  
  return 0;
}

}
