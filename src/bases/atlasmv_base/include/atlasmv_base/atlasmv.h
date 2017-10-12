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
 * @brief Includes, global vars, funtion prototypes, etc.
 */

#ifndef _ATLASMV_H_
#define _ATLASMV_H_

#define _DO_NOT_USE_OPENCV_ 0
#define _DO_NOT_USE_CARMEN_ 0

//####################################################################
// Includes: no need to include headers from carmen modules carmen.h #
// already does			#####################################
//####################################################################

#include "c_atlasmv.h"

#include <pthread.h>
#include <strings.h>
#include <math.h>
#include <queue>
#include <vector>
#include <atlasmv_base/AtlasmvLightsCommand.h>
#include <atlasmv_base/AtlasmvVertSignsCommand.h>
#include <atlasmv_base/AtlasmvMotionCommand.h>
#include <atlasmv_base/AtlasmvStatus.h>

using namespace std;

template <class cmd_type>
class command_queue_priority
{
	typename std::map<int,cmd_type> message_map;
	
	typename std::map<int,cmd_type>::iterator it;
	
	public:
		void maintenance(void)
		{
			for(it=message_map.begin();it!=message_map.end();it++)
				if(ros::Time::now().toSec() > (*it).second->lifetime)
					message_map.erase(it);
		}
		
		void push_msg(cmd_type msg)
		{
			msg->lifetime=ros::Time::now().toSec()+msg->lifetime;
			
			it=message_map.find(msg->priority);
			
			if(it==message_map.end())
				message_map[msg->priority]=msg;
			else
			{
				message_map.erase(it);
				message_map[msg->priority]=msg;
			}
		}
		
		cmd_type top_msg(void)
		{
			maintenance();
			
			int max_priority=-1;
			typename std::map<int,cmd_type>::iterator it_max;
			
			for(it=message_map.begin();it!=message_map.end();it++)
				if((*it).second->priority > max_priority)
				{
					max_priority=(*it).second->priority;
					it_max=it;
				}
			
			if(message_map.size())
				return (*it_max).second;
			
			cmd_type null_ptr;
			return null_ptr;
		}
};


typedef struct
{
	///Position in X and Y in the global reference
	double X,Y;
	///Linear velocity
	double V;
	///Steering angle
	double SA;
	///Heading angle
	double HA;
	///Wheelbase
	double L;
	///Distance traveled
	double S;
	///Curve radius
	double R;
}t_motion_model;

//####################################################################
// Typedefs: required typedefs are declared here. All declared types #
// should have TYPE_[the name] 			###############################
//####################################################################

//####################################################################
// Prototypes:	for private functions can be declared #################
// in atlasmv.c or preferably in atlasmv_functions.c  ##
//####################################################################
int main(int argc, char **argv);
void atlasmv_shutdown_module(int x);
void atlasmv_help(bool debug);
// void handler_param_daemon(void);
// void handler_speed_gain(TYPE_des_sysparam* sys, c_atlasmv* robot, TYPE_executionflags *);
// void handler_motion(custom_data_t *);
// bool CheckBrakeToggle(void);

double percentage2real_velocity(double percentage);
int increment_motion_model(t_motion_model*model,double dt);
int update_motion_model(t_motion_model*model,double velocity,double steering_angle);
int initialize_motion_model(t_motion_model*model,double X0,double Y0,double orientation,double wheelbase);

//####################################################################
// Global vars: They will be in the scope of modulename.c main's #####
// otherwise are declared extern #####################################
//####################################################################

//####################################################################
// Global vars: They will be in the scope of modulename.c main's #####
// otherwise are declared extern #####################################
//####################################################################
//mike confusion, uses a message structure to install parameters. message and parameters should not be different?


//a thread for each communication
pthread_t des_thread, dioc_thread, servos_thread;
int des_iret, dioc_iret, servos_iret;
pthread_mutex_t speed_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t dir_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t brk_mutex = PTHREAD_MUTEX_INITIALIZER; 
pthread_mutex_t dioc_mutex = PTHREAD_MUTEX_INITIALIZER; 
pthread_mutex_t command_mutex = PTHREAD_MUTEX_INITIALIZER; 
pthread_mutex_t count_des_errors_mutex;
pthread_mutex_t speed_check_mutex;
int speed_check;
pthread_mutex_t brake_check_mutex;
int brake_check;
int count_des_errors;


TYPE_atlasmv_public_params params; //this is a structure
c_timer timer;
t_motion_model motion_model;

double speed, dir, brake;
char new_command_received;

atlasmv_base::AtlasmvStatusPtr atlasmv_status;
atlasmv_base::AtlasmvMotionCommandPtr atlasmv_motion;
atlasmv_base::AtlasmvVertSignsCommandPtr atlasmv_vert_signs;
atlasmv_base::AtlasmvLightsCommandPtr atlasmv_lights;



TYPE_atlasmv_public_params atlasmv_details;
c_atlasmv* atlasmv;

command_queue_priority<atlasmv_base::AtlasmvMotionCommandPtr> command_queue;

#endif
