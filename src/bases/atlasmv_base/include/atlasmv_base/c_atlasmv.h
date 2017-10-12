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
/** @file
 * @brief c_atlasmv.h file for this module. Includes, global vars, function prototypes, etc.
 */

#ifndef _CLASS_ATLASMV_H_
#define _CLASS_ATLASMV_H_

#define _DO_NOT_USE_OPENCV_ 0
#define _DO_NOT_USE_CARMEN_ 0

//####################################################################
// Includes: no need to include headers from carmen modules carmen.h #
// already does			#####################################
//####################################################################

#include <pthread.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ros/ros.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <atlasmv_base/timer.h>
// #include <class_hitec5980gs.h>
#include <hitec5980sg/hitec5980sg.h>
#include <atlasmv_base/class_dioc.h>
#include <atlasmv_base/des70_10.h>
#include <iostream>

#include <atlasmv_base/AtlasmvLightsCommand.h>
#include <atlasmv_base/AtlasmvMotionCommand.h>
#include <atlasmv_base/AtlasmvVertSignsCommand.h>
#include <atlasmv_base/AtlasmvStatus.h>

using namespace std;

typedef enum
{
	ENUM_ERR_NONE = 0,				/// nothing relevant happens
	ENUM_ERR_READ_DIR = 3,			/// direction servo is not responding
	ENUM_ERR_OVER_CURRENT = 4,		/// short circuit at motor windings. batterry unable to supply acceleration curretn. Overtemperature, damaged power stage
	ENUM_ERR_OVER_VOLTAGE = 5,		/// The power supply is too high
	ENUM_ERR_OVER_SPEED = 6,		///	Engine rpm too high (above 30000rpm)
	ENUM_ERR_SUPPLY_VOLT_LOW = 7,	/// The voltage is too low (under 10V)
	ENUM_ERR_ANG_DET = 8,			/// The angle measured between encoder and hall sensors is to high
	ENUM_ERR_ENC_RES = 9			/// Param Nb 20 - "Encoder Resolution" out of range
}ENUM_ERR;

typedef enum
{
	MANDATORY_MEDIUM_LIGTHS = 1,
	MANDATORY_BUS_LANE = 2,
	WARNING_ROAD_NARROWS = 3,
	WARNING_DIP_AHEAD = 4,
	INFORMATION_HOSPITAL = 5,
	INFORMATION_REC_SPEED_60 = 6,
	NO_VERT_SIGN=0,
}TYPE_ENUM_SIGNS;

/**
 * @brief typedef that includes all params that define atlasmv robot
 */
typedef struct details{
	int cycle_freq;
	double sleep_time;

	double max_forward_speed;
	double max_backward_speed;
	double brake_speed_dif;
	//servo device ids
	int servo_direction_id;
	int servo_brake_id;
	//device paths
	std::string com_servos;
	std::string com_dioc;
	std::string com_des;
	//phisical aspects
	double length;
	double width;
	double back_wheel_diam;
	double wheelaxisdistance;
	double maximum_dir;
	double minimum_dir;
	double maximum_brk;
	double minimum_brk;
	double transmission_relation;
	int min_pulse_brk;
	int max_pulse_brk;
	int min_pulse_dir;
	int max_pulse_dir;
	double brk_time_no_maxon;
	//servoamplifier aspects
	TYPE_des_sysparam sDES_sysparam;
	int des_frq_chk_errors;
	
	friend ostream& operator<< (ostream &o, const details &i)
	{
		return o
		<< "cycle_freq: "<< i.cycle_freq <<endl
		<< "sleep_time: "<< i.sleep_time<<endl
		<< "com_servos: "<< i.com_servos<<endl
		<< "com_dioc: "<< i.com_dioc<<endl
		<< "com_des: "<< i.com_des<<endl;
	}
}TYPE_atlasmv_public_params;

/** 
 * @brief class that defines everything related with the atlasmv robot interface. this class only deals with SI units (m/s, rad/s, seconds and so on)
 */
class c_atlasmv{
	private:
		//variables used that are connected with class running

		pthread_mutex_t c_atlasmv_robot_details_mutex;
		TYPE_atlasmv_public_params *robot_details;
		pthread_mutex_t flags_mutex;
		bool debug_mode;
// 		TYPE_executionflags* flags;
		char get_executionflags_debug();
		c_timer *timer;
		pthread_mutex_t errors_mutex;
		int get_errors();
		int set_errors(int);
		int errors;


		//variables that are connected with des
		pthread_mutex_t max_forward_speed_mutex;
		double max_forward_speed;
		pthread_mutex_t max_backward_speed_mutex;
		double max_backward_speed;
		pthread_mutex_t brake_speed_dif_mutex;
		double brake_speed_dif;
		//device paths
		pthread_mutex_t com_des_mutex;
		std::string com_des;
		pthread_mutex_t porthandler_des_mutex;
		int* porthandler_des;
		//phisical aspects
		pthread_mutex_t length_mutex;
		double length;
		pthread_mutex_t width_mutex;
		double width;
		pthread_mutex_t back_wheel_diam_mutex;
		double back_wheel_diam;
		pthread_mutex_t wheelaxisdistance_mutex;
		double wheelaxisdistance;
		pthread_mutex_t transmission_relation_mutex;
		double transmission_relation;
		pthread_mutex_t sDES_sysparam_mutex;
		TYPE_des_sysparam sDES_sysparam;
		pthread_mutex_t rpm_mutex;
		int actual_rpm;
		int req_rpm;
		pthread_mutex_t linear_speed_mutex;
		double actual_speed;
		double req_speed;
		pthread_mutex_t des_initialized_mutex;
		char des_initialized;
		pthread_mutex_t brk_time_no_maxon_mutex;
		double brk_time_no_maxon;

		/************************************************************
		*				related with servo control					*
		************************************************************/
		//--- srv_var
		pthread_mutex_t maximum_dir_mutex;
		double maximum_dir;
		pthread_mutex_t minimum_dir_mutex;
		double minimum_dir;
		pthread_mutex_t maximum_brk_mutex;
		double maximum_brk;
		pthread_mutex_t minimum_brk_mutex;
		double minimum_brk;
		pthread_mutex_t com_servos_mutex;
		std::string com_servos;
		pthread_mutex_t actual_dir_mutex;
		double actual_dir;
		pthread_mutex_t actual_brk_mutex;
		double actual_brk;
		pthread_mutex_t servo_initialized_mutex;
		char servo_initialized;
		pthread_mutex_t min_pulse_brk_mutex;
		int min_pulse_brk;
		pthread_mutex_t max_pulse_brk_mutex;
		int max_pulse_brk;
		pthread_mutex_t min_pulse_dir_mutex;
		int min_pulse_dir;
		pthread_mutex_t max_pulse_dir_mutex;
		int max_pulse_dir;	
		//servo device ids
		pthread_mutex_t servo_direction_id_mutex;
		int servo_direction_id;
		pthread_mutex_t servo_brake_id_mutex;
		int servo_brake_id;


		float slope_rad2pulse;
		float bias_rad2pulse;
		float slope_pulse2rad;
		float bias_pulse2rad;
		hitec_5980SG *servo;
		//float direction_ang;
		//float brake_ang;
		
		//--- srv_func
		int init_com_servo();

		/************************************************************
		*		related with discrete input and ouptup board		*
		************************************************************/
		//--- sens_var
		pthread_mutex_t com_dioc_mutex;
		std::string com_dioc;
		class_dioc *dioc;
		pthread_mutex_t dioc_initialized_mutex;
		char dioc_initialized;
		pthread_mutex_t sign_mutex;
		int prev_sign;
		pthread_mutex_t lights_mutex;
		int prev_lights;
		pthread_mutex_t cross_sensor_mutex;
		int cross_sensor;
		//--- sens_func
		int init_com_dioc();
		pthread_mutex_t robot_status_mutex;
		int turnright;		///bool
		int turnleft;		///bool
		int headlights;		///bool
		int taillights;		///bool
		int reverselights;	///bool
		int vert_sign;		///enum

		/************************************************************
		*				related with DES servoamplifier				*
		************************************************************/
		//--- des_var
		bool init;
		//char* com_des;
		double velocity;

		//--- des_func
		int init_com_des();
		
		int rad2pulses(double rad);
		double pulses2rad(int pulses);

	public:
		c_atlasmv(bool debug_);//Y
		int set_atlasmv_details(TYPE_atlasmv_public_params *);
		int initialize_robot(void);// not totally
		int new_robot_state(atlasmv_base::AtlasmvMotionCommand&,atlasmv_base::AtlasmvVertSignsCommand&,atlasmv_base::AtlasmvLightsCommand&);
		int update_robot_status(atlasmv_base::AtlasmvStatus&);

		////////////////////////////////////////////
		//-------------- des_func  ---------------//
		////////////////////////////////////////////
		int check_des(void);

		//functions


		double map_linear(double in,std::vector<std::pair<double,double> >& cp)
		{
			//first está nas coordenadas de in
			//second está nas coordenadas de out
			
			double out=NAN;
			
			for(uint i=0;i<cp.size()-1;i++)
			{
				if( in >= cp[i].first && in <= cp[i+1].first)
				{
					//encontrei o intervalo certo
					double m = (cp[i+1].second - cp[i].second)/(cp[i+1].first - cp[i].first);
					double b = cp[i+1].second -m*cp[i+1].first;
					
					out=m*in+b;
					break;
				}
			}
			
			if(isnan(out)){
				cout<<"Warning: Input argument: "<<in<<" is out of bounds, ["<<cp[0].first<<","<< cp[cp.size()-1].first<<"]"<<endl;
                                return 0;
                        }
                        else{
                          return out;
                        }
		}


		//char *com_des;
		int restart_des();
		int set_speed(double *);
		int read_speed(int);

		void set_com_des(std::string);//Y
		std::string get_com_des();
		void set_porthandler_des(int );
		int get_porthandler_des();
		void set_linear_speed(double*, double *);
		void get_linear_speed(double *, double *);
		void set_rpm(int *, int *);
		void get_rpm(int *, int *);
		void set_max_forward_speed(double *);
		double get_max_forward_speed();
		void set_max_backward_speed(double *);
		double get_max_backward_speed();
		void set_brake_speed_dif(double *);
		double get_brake_speed_dif();
		void set_length(double *);
		double get_length();
		void set_width(double *);
		double get_width();
		void set_back_wheel_diam(double *);
		double get_back_wheel_diam();
		void set_wheelaxisdistance(double *);
		double get_wheelaxisdistance();
		void set_transmission_relation(double *);
		double get_transmission_relation();
		void set_sDES_sysparam(TYPE_des_sysparam *);
		void get_sDES_sysparam(TYPE_des_sysparam *);
		void set_brk_time_no_maxon(double *);
		double get_brk_time_no_maxon();
		//void set_actual_rpm(int *);
		//void get_actual_rpm(int *);
		//void set_req_rpm(int *);
		//void get_req_rpm(int *);
		//void set_actual_speed(double *);
		//void get_actual_speed(double *);
		//void set_req_speed(double *);
		//void get_req_speed(double *);
		void set_des_initialized(char);
		char get_des_initialized();

		//--- srv_func
		int set_direction(double *);
		int read_direction(double *);
		double get_actual_dir();
		void set_actual_dir(double *);
		double get_actual_brk();
		void set_actual_brk(double *);
		int set_brake(double *);
		//int read_brake(double *brake, int *)=NUL;
		int read_brake(double *brk, int *num_pulses=NULL);


		void set_com_servo(std::string);//Y
		std::string get_com_servo();
		void set_dir_srv_id(unsigned char);
		char get_dir_srv_id();
		void set_brk_srv_id(unsigned char);
		char get_brk_srv_id();
		void set_maximum_brk_angle(double *);
		double get_maximum_brk_angle();
		void set_minimum_brk_angle(double *);
		double get_minimum_brk_angle();
		void set_maximum_dir_angle(double *);
		double get_maximum_dir_angle();
		void set_minimum_dir_angle(double *);
		double get_minimum_dir_angle();
		void set_min_pulse_brk(int *);
		int get_min_pulse_brk();
		void set_max_pulse_brk(int *);
		int get_max_pulse_brk();
		void set_min_pulse_dir(int *);
		int get_min_pulse_dir();
		void set_max_pulse_dir(int *);
		int get_max_pulse_dir();
		void set_servo_initialized(char);
		char get_servo_initialized();

		//--- sens_func
		int set_lights(atlasmv_base::AtlasmvVertSignsCommand&,atlasmv_base::AtlasmvLightsCommand&);
		int read_cross_sensor();
		//char *com_dioc;
		void set_com_dioc(std::string);
		std::string get_com_dioc();
		void set_dioc_initialized(char);
		char get_dioc_initialized();
		void set_cross_sensor(int);
		int get_cross_sensor();
		int get_lights(atlasmv_base::AtlasmvStatus&);
		
		
		~c_atlasmv();//Y
};

#endif
