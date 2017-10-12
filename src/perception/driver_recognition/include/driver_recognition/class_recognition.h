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
/** \brief Nodelet that recieves ros msg's from a bag file and then prints to a text file,  
 * 
 *  \file
 *  \author André Oliveira
 *  \date May 2012
 */

#ifndef _CLASS_RECOGNITION_H_
#define _CLASS_RECOGNITION_H_

#include <stdio.h>
#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <string.h>
// #include <atlascar_base/AtlascarStatus.h>
// #include <atlascar_base/AtlascarVelocityStatus.h>
// #include <atlascar_base/AtlascarPartialStatus.h>
#include "ros/file_log.h"
#include "rosgraph_msgs/Log.h"
#define PFLN printf("Line = %d\n", __LINE__);
#include <tf/transform_listener.h>


using namespace std;

/** \class class_recognition
 *  \brief A class to store the structures used on the nodes
 */
class class_recognition
{
	public:
		/** \struct TYPE_msg_partial
		  *  \brief to store the variables of driver monitoring arduino
		  */
		struct TYPE_msg_partial
		{
		int lights_high;
		int lights_medium;
		int ignition;
		int lights_left;
		int lights_right;
		int danger_lights;
		int horn;
		int throttle;
		int brake;
		int clutch;
		float velocity;
		float steering_wheel;
		float rpm;
		
		friend ostream& operator<< (ostream &o, const TYPE_msg_partial &i)
			{
				return o
				<<"||||||||msg_partial||||||"<<endl
				<< "lights_high: "<< i.lights_high<<endl
				<< "lights_medium: "<< i.lights_medium<<endl
				<< "ignition: "<< i.ignition<<endl
				<< "lights_left: "<< i.lights_left<<endl
				<< "lights_right: "<< i.lights_right<<endl
				<< "danger_lights: "<< i.danger_lights<<endl
				<< "horn: "<< i.horn<<endl	
				<< "throttle: "<< i.throttle<<endl
				<< "brake: "<< i.brake<<endl
				<< "clutch: "<< i.clutch<<endl
				<< "velocity: "<< i.velocity<<endl
				<< "steering_wheel: "<< i.steering_wheel<<endl
				<< "rpm: "<< i.rpm<<endl;
			}
		};

		


		/** \struct TYPE_msg_bags
		  *  \brief to store the name of the bag file played 
		  */		
		struct TYPE_msg_bags
		{
		string name_file;
		double   bag_time;
		
		
				
		friend ostream& operator<< (ostream &o, const TYPE_msg_bags &i)
		{
			return o
			<<"||||||||msg_plc||||||"<<endl
			<< "name_file: "<< i.name_file<<endl
 			<< "bag_time: "<< i.bag_time<<endl;
		}
			
		};
		
		
		
		/**
		@brief Class constructor.
		*/
		class_recognition()
		{ 		};
		
		/**
		@brief Class destructor.
		*/
		~class_recognition()
		{		};
		
		
	private:
		
		
		
		
};

#endif
