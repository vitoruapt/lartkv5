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
 * @file state_machine.cpp
 * @brief Planar scan generator
 * @author Miguel Oliveira
 * @version v0
 * @date 2012-02-29
 */

#ifndef _STATE_MACHINE_CPP_
#define _STATE_MACHINE_CPP_

#include "follow_pedestrian.h"

void state_machine(void)
{


	a_h = (((double)rand())/(double)RAND_MAX);




	///Draw the 
	//if (status.CONTROL_CAR_SPEED)

	//sprintf(text,"%s",status.CONTROL_CAR_SPEED?"CAR SPEED AUTOMATIC CONTROL (press A to change)":"CAR SPEED MANUAL CONTROL (press A to change)");
	//draw_text_on_glwindow(t_viewpoint.glwindow.width/2, t_viewpoint.glwindow.height-20,text);

	//sprintf(text,"(press R to abort tracking)");
	//draw_text_on_glwindow(t_viewpoint.glwindow.width/2, t_viewpoint.glwindow.height-20*2,text);

	//sprintf(text,"%s",do_execute_tutorial?"Tutorial On (press t to change)":"Tutorial Off (press t to change)");
	//draw_text_on_glwindow(t_viewpoint.glwindow.width/2, t_viewpoint.glwindow.height-20*3,text);



	//sprintf(text,"Direction: %f",hc.direction);
	//draw_text_on_glwindow(10, t_viewpoint.glwindow.height-60,text);
	//sprintf(text,"Speed: %f",hc.speed);
	//draw_text_on_glwindow(10, t_viewpoint.glwindow.height-75,text);

	//sprintf(text,"Lights: Left %s Right %s Max %s Med %s ",hc.lights_left?"On":"Off",hc.lights_right?"On":"Off",hc.lights_high?"On":"Off",hc.lights_medium?"On":"Off");
	//draw_text_on_glwindow(10, t_viewpoint.glwindow.height-90,text);


	///some local variables

	//if (status.state==TRACKING || status.state==TRACKING_NOT_SAFE) //if were tracking someone, lets draw it
	//{
	//}

	printf("status.target_found=%d\n",status.target_found);
	printf("status.last_id=%d\n",status.last_id);

	/// Tracking state machine
	status.target_found=false;
	int should_move=false;
	double dist_to_target=1000;



	print_status(&status);
	/// ---------------------------------------------------------------------------
	/// STATE MACHINE
	/// ---------------------------------------------------------------------------
	switch(status.state)
	{
		/// ---------------------------------------------------------------------------
		case INITIALISE: /// STATE INITIALIZE
			/// ---------------------------------------------------------------------------

			///--- TIMER UPDATES --- //

			/// --- STATE ACTIONS ---//
			//bring the ptu to home position
			foveationcontrol_home(); 

			//setting up the high command message
			status.direction = DIRECTION_IN_FRONT;
// 			set_high_command_msg_direction_speed(command_msg, status.direction, SPEED_STOP);
// 			set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);


			/// --- TRANSITIONS ---//
			status.previous_state=INITIALISE; //unconditional transition to searching state
			status.state=SEARCHING;
			/// --- TRANSITION ACTIONS ---//
			//none here

			break;
			/// ---------------------------------------------------------------------------
		case SEARCHING: /// STATE SEARCHING
			/// ---------------------------------------------------------------------------

			///--- TIMER UPDATES --- //
			if (status.previous_state!=SEARCHING) //update start of SEARCHING state tic
			{
				status.tic_searching=ros::Time::now();
			}

			/// --- STATE ACTIONS ---//
			//search for a target
			status.target_found = search_for_new_target(&targets, &status,&search_area);
			printf("status.target_found=%d\n",status.target_found);
			status.last_id=status.target_id;

			play_sound(4, 10);
			foveationcontrol_sweeping();

			//setting up the high command message
// 			status.direction = command_msg.steering_wheel;
// 			set_high_command_msg_direction_speed(command_msg, status.direction, SPEED_STOP);
// 			set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);

			/// --- TRANSITIONS ---//
			if(status.target_found && !is_safe_using_lasers()) //if a target but safe zone not clear we transit to TRACKING_NOT_SAFE
			{
				status.previous_state=SEARCHING;
				status.state=TRACKING_NOT_SAFE;
				/// --- TRANSITION ACTIONS ---//
				play_sound(2, 10);
			}
			else if(status.target_found) //if a target is found we transit to tracking
			{
				status.previous_state=SEARCHING;
				status.state=TRACKING;
				/// --- TRANSITION ACTIONS ---//
				play_sound(0, -1);
			}
			break;
			/// ---------------------------------------------------------------------------
		case TRACKING: /// STATE TRACKING
			/// ---------------------------------------------------------------------------
			///--- TIMER UPDATES --- //
			if (status.previous_state!=TRACKING) //update start of SEARCHING state tic
			{
				status.tic_tracking=ros::Time::now();
			}

			/// --- STATE ACTIONS ---//
			foveationcontrol(&status);//do foveationcontrol
			status.target_found=get_new_target_information(&targets, &status);

			//setting up the high command message
			play_sound(1, 15);

			//send pedestrian marker
			

			status.direction = atan2(status.current_y,status.current_x);
// 			set_high_command_msg_direction_speed(command_msg, status.direction, SPEED_MOVE_SLOW);
// 			set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_ON, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);

			/// --- TRANSITIONS ---//
			if(status.reset_tracking==true)//user ordered a reset of tracking
			{
				status.previous_state=TRACKING;
				status.state=SEARCHING;
				status.reset_tracking = false;
				/// --- TRANSITION ACTIONS ---//
				play_sound(5, -1);
			}
			else if(status.target_found==false || status.current_x<3.0)//target was lost
			{
				status.previous_state=TRACKING;
				status.state=TARGET_LOST;
				/// --- TRANSITION ACTIONS ---//
				play_sound(5, -1);

			}else if (!is_safe_using_lasers())
			{
				status.previous_state=TRACKING;
				status.state=TRACKING_NOT_SAFE;
				/// --- TRANSITION ACTIONS ---//
				if (dist_to_target<6)
					play_sound(6, -1);
				else
					play_sound(2, -1);
			}
			break;
			/// ---------------------------------------------------------------------------
		case TRACKING_NOT_SAFE: /// TRACKING_NOT_SAFE
			/// ---------------------------------------------------------------------------
			///--- TIMER UPDATES --- //
			if (status.previous_state!=TRACKING_NOT_SAFE) //update start of SEARCHING state tic
			{
				status.tic_tracking_not_safe=ros::Time::now();
			}

			/// --- STATE ACTIONS ---//
			status.target_found = get_new_target_information(&targets, &status);
			foveationcontrol(&status);//do foveationcontrol

			if (status.target_found)
			{
				dist_to_target = sqrt(pow(status.current_x-2.7,2) + pow(status.current_y,2));
				printf("status.target_id=%d\n",status.target_id);
				printf("dist_to_target=%f\n",dist_to_target);
				if (dist_to_target<4)
					play_sound(6, 10);
				else
					play_sound(2, 10);
			}


			//setting up the high command message
			status.direction = atan2(status.current_y,status.current_x);
// 			set_high_command_msg_direction_speed(command_msg, status.direction, SPEED_STOP);
// 			set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_ON, LIGHT_ON, LIGHT_ON, LIGHT_ON);

			/// --- TRANSITIONS ---//
			if(status.reset_tracking==true)//user ordered a reset of tracking
			{
				status.previous_state=TRACKING_NOT_SAFE;
				status.state=SEARCHING;
				status.reset_tracking = false;
				/// --- TRANSITION ACTIONS ---//

			}
			else if(status.target_found==false || status.current_x<3.0)//target was lost
			{
				status.previous_state=TRACKING_NOT_SAFE;
				status.state=TARGET_LOST;
				/// --- TRANSITION ACTIONS ---//
				play_sound(5, -1);

			}
			else if (is_safe_using_lasers())
			{
				int tmp = status.previous_state;
				status.previous_state=TRACKING_NOT_SAFE;
				status.state=TRACKING;
				/// --- TRANSITION ACTIONS ---//
				if (tmp!=SEARCHING)
				{
					play_sound(3, -1);
				}
				else
				{
				}
			}

			break;
			/// ---------------------------------------------------------------------------
		case TARGET_LOST: /// TARGET_LOST
			/// ---------------------------------------------------------------------------
			///--- TIMER UPDATES --- //


			/// --- STATE ACTIONS ---//			
			foveationcontrol_home();

			//setting up the high command message
// 			set_high_command_msg_direction_speed(command_msg, command_msg.steering_wheel, SPEED_STOP);
// 			set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);


			/// --- TRANSITIONS ---//			
			status.previous_state=TARGET_LOST; 
			status.state=SEARCHING; //unconditional transition to SEARCHING


			break;
			/// ---------------------------------------------------------------------------
		default: ///UNKNOWN STATE
			/// ---------------------------------------------------------------------------

			ROS_ERROR("WARNING DANGER unknown state in state machine abort demonstration");
			//setting up the high command message
// 			set_high_command_msg_direction_speed(command_msg, command_msg.steering_wheel, SPEED_STOP);
// 			set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);
			break;
	}

	//if (!status.CONTROL_CAR_SPEED)
	//{
	//hc.speed = SPEED_STOP;
	//}

// 	command_msg.header.stamp = ros::Time::now();
// 	command_pub.publish(command_msg);
}


#endif
