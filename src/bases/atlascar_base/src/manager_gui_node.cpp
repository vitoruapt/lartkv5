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
\brief Atlascar Manager GUI node
*/

#include <ros/ros.h>

#include "manager_gui.h"
#include <gtkmm/application.h>

#include <signal.h>
#include <ros/xmlrpc_manager.h>

using namespace std;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile request_shutdown = 0;

void sigHandler(int s)
{
	request_shutdown=1;
}

//Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		request_shutdown=1; // Set flag
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"manager_gui",ros::init_options::NoSigintHandler);
	signal(SIGINT,sigHandler);
	
	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
  
	Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv, "org.gtkmm.example");
	
	ManagerGui gui(&request_shutdown);
	
	return app->run(gui);
}
