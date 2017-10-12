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
#ifndef _CLASS_COLISION_TIME_H_
#define _CLASS_COLISION_TIME_H_


/** \brief This is the class definition of file colision_time.cpp
 *  \file class_colision_time.h
 *  \author André Oliveira
 *  \date May 2012
 */
#include <stdio.h>
#include <ros/ros.h>
#include <mtt/TargetListPC.h>
#include <time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <points_from_volume/points_from_volume.h>
// #include <atlascar_base/AtlascarStatus.h>
// #include <atlascar_base/AtlascarVelocityStatus.h>
// #include <atlascar_base/AtlascarPartialStatus.h>
#include <visualization_msgs/Marker.h>

#include "ros/file_log.h"
#include "rosgraph_msgs/Log.h"

#define PFLN printf("Line = %d\n", __LINE__);

using namespace std;
using namespace pcl;

/** \class class_colision_time
 *  \brief A class to detect danger situations while driving 
 */
class class_colision_time
{
	public:
		
		struct TYPE_msg_mtt
		{
		int id;
		float x;
		float y;
		float z;
		float v_x;
		float v_y;
		float v_z;
		
		friend ostream& operator<< (ostream &o, const TYPE_msg_mtt &i)
			{
				return o
				<<"||||||||msg_mtt||||||"<<endl
				<< "id: "<< i.id<<endl
				<< "x: "<< i.x<<endl
				<< "y: "<< i.y<<endl
				<< "z: "<< i.z<<endl
				<< "v_x: "<< i.v_x<<endl
				<< "v_y: "<< i.v_y<<endl
				<< "v_z: "<< i.v_z<<endl;	
			}
		
		};

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
				<< "clutch: "<< i.clutch<<endl;	
			}
		};

		struct TYPE_msg_velocity
		{
		float counting;
		float pulses_sec;
		float revolutions_sec;
		float velocity;
		
		friend ostream& operator<< (ostream &o, const TYPE_msg_velocity &i)
			{
				return o
				<<"||||||||msg_velocityl||||||"<<endl
				<< "counting: "<< i.counting<<endl
				<< "pulses_sec: "<< i.pulses_sec<<endl
				<< "revolutions_sec: "<< i.revolutions_sec<<endl
				<< "velocity: "<< i.velocity<<endl;
			}		
		};


		struct TYPE_msg_plc
		{
		float steering_wheel;
		float rpm;
		
		
// 		friend ostream& operator<< (ostream &o, const t_parameters &i)
		friend ostream& operator<< (ostream &o, const TYPE_msg_plc &i)
		{
			return o
			<<"||||||||msg_plc||||||"<<endl
			<< "steering_wheel: "<< i.steering_wheel<<endl
			<< "rpm: "<< i.rpm<<endl;
		}
			
		};

		
		struct TYPE_msg_bags
		{
		string name_file;
		
		//double   sit_begin;
		//double   duration;
// 		double   sit_end_1;
		double   bag_time;
		
		
		string sit_type_1;
		int 	sit_1;
		double lst_obj_sit_1;
		int internal_1;
		double   sit_begin_1;
		double   duration_1;
		double   sit_end_1;
		
		string sit_type_2;
		int 	sit_2;
		double lst_obj_sit_2;
		int internal_2;
		double   sit_begin_2;
		double   duration_2;
		double   sit_end_2;
		
		string sit_type_3;
		int 	sit_3;
		double lst_obj_sit_3;
		int internal_3;
		double   sit_begin_3;
		double   duration_3;
		double   sit_end_3;
		
		string sit_type_4;
		int 	sit_4;
		float lst_obj_sit_4;
		int internal_4;
		double   sit_begin_4;
		double   duration_4;
		double   sit_end_4;
		
		
		
		friend ostream& operator<< (ostream &o, const TYPE_msg_bags &i)
		{
			return o
			<<"||||||||msg_plc||||||"<<endl
			<< "name_file: "<< i.name_file<<endl
// 			<< "sit_type: "<< i.sit_type<<endl
// 			<< "sit_begin: "<< i.sit_begin<<endl
// 			<< "duration: "<< i.duration<<endl
// 			<< "sit_end: "<< i.sit_end<<endl
 			<< "bag_time: "<< i.bag_time<<endl;
		}
			
		};
		
		
		
		/**
		@brief Class constructor.
		*/
		class_colision_time()
		{ 		};
		
		/**
		@brief Class destructor.
		*/
		~class_colision_time()
		{		};
		
		
// 		void topic_callback_partial(atlascar_base::AtlascarPartialStatus AtlascarPartialStatus_msg);
// // 		
// 		void topic_callback_velocity(atlascar_base::AtlascarVelocityStatus AtlascarVelocityStatus_msg);
// 		
// 		void topic_callback_plc_status(atlascar_base::AtlascarStatus AtlascarStatus_msg);
		
// 		int cria_txt(void);
// 		char* TimeString();
		
		
	private:
		
		
		
		
};

#endif
