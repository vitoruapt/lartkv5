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
 * @brief class_atlasmv.cpp file for this module. This is where main is defined and etc.
 */

#include <atlasmv_base/c_atlasmv.h>

/** 
 * @brief initializes pointers and some veriables
 * 
 * @param debug debug mode
 */
c_atlasmv::c_atlasmv(bool debug)
{
	debug_mode=debug;
	
	robot_details = NULL;

	timer = new c_timer;
	prev_sign = 123434;
	prev_lights = 122344;


	
	//initialize servo variables
	slope_rad2pulse = -1473.32;
	bias_rad2pulse = 1480;
	slope_pulse2rad = -0.000678;
	bias_pulse2rad = 1.0035;
	servo_initialized=0;

	
	actual_dir = 0.0;
	actual_brk=0.0;
	//direction_ang = 0.0;
	//brake_ang = 0.0;

	dioc = NULL;
	dioc_initialized=0;

	//initialize DES variables
	porthandler_des = new int;
	*porthandler_des=0;
	//sDES_sysparam = NULL;
	des_initialized=0;
	//dioc_initialized=0;
	pthread_mutex_init(&errors_mutex, NULL);
	pthread_mutex_init(&flags_mutex, NULL);

	pthread_mutex_init(&c_atlasmv_robot_details_mutex, NULL);
	/************************************************************
	*				related with DES servoamplifier				*
	************************************************************/
	pthread_mutex_init(&flags_mutex, NULL);
	pthread_mutex_init(&max_forward_speed_mutex, NULL);
	pthread_mutex_init(&max_backward_speed_mutex, NULL);
	pthread_mutex_init(&brake_speed_dif_mutex, NULL);
	pthread_mutex_init(&servo_direction_id_mutex, NULL);
	pthread_mutex_init(&servo_brake_id_mutex, NULL);
	pthread_mutex_init(&com_des_mutex, NULL);
	pthread_mutex_init(&porthandler_des_mutex, NULL);
	pthread_mutex_init(&length_mutex, NULL);
	pthread_mutex_init(&width_mutex, NULL);
	pthread_mutex_init(&back_wheel_diam_mutex, NULL);
	pthread_mutex_init(&wheelaxisdistance_mutex, NULL);
	pthread_mutex_init(&maximum_dir_mutex, NULL);
	pthread_mutex_init(&minimum_dir_mutex, NULL);
	pthread_mutex_init(&maximum_brk_mutex, NULL);
	pthread_mutex_init(&minimum_brk_mutex, NULL);
	pthread_mutex_init(&transmission_relation_mutex, NULL);
	pthread_mutex_init(&min_pulse_brk_mutex, NULL);
	pthread_mutex_init(&max_pulse_brk_mutex, NULL);
	pthread_mutex_init(&min_pulse_dir_mutex, NULL);
	pthread_mutex_init(&max_pulse_dir_mutex, NULL);
	pthread_mutex_init(&sDES_sysparam_mutex, NULL);
	pthread_mutex_init(&rpm_mutex, NULL);
	pthread_mutex_init(&linear_speed_mutex, NULL);
	pthread_mutex_init(&des_initialized_mutex, NULL);

	/************************************************************
	*				related with servo control					*
	************************************************************/
	pthread_mutex_init(&com_servos_mutex, NULL);
	pthread_mutex_init(&actual_dir_mutex, NULL);
	pthread_mutex_init(&actual_brk_mutex, NULL);
	pthread_mutex_init(&servo_initialized_mutex, NULL);
	pthread_mutex_init(&min_pulse_brk_mutex, NULL);
	pthread_mutex_init(&max_pulse_brk_mutex, NULL);
	pthread_mutex_init(&min_pulse_dir_mutex, NULL);
	pthread_mutex_init(&max_pulse_dir_mutex, NULL);
	pthread_mutex_init(&brk_time_no_maxon_mutex, NULL);

	/************************************************************
	*		related with discrete input and ouptup board		*
	************************************************************/
	pthread_mutex_init(&com_dioc_mutex, NULL);
	pthread_mutex_init(&dioc_initialized_mutex, NULL);
	pthread_mutex_init(&sign_mutex, NULL);
	pthread_mutex_init(&lights_mutex, NULL);
	pthread_mutex_init(&cross_sensor_mutex, NULL);
};

/** 
 * @brief delete pointers and close communications safely
 */
c_atlasmv::~c_atlasmv()
{
	printf("Shutting down ports & robot...");

	stopDES(*porthandler_des,debug_mode);

	//delete sensors;

	delete timer;

	//---	SERVOS	---//
	delete servo;

	//---	SENS & LIGHTS	---//
	delete dioc;

	//---	DES params	---//
	//delete sDES_sysparam;
	delete porthandler_des;

	printf("Done\n");
};

void c_atlasmv::set_brk_time_no_maxon(double *a)
{
	pthread_mutex_lock(&brk_time_no_maxon_mutex);
	brk_time_no_maxon = *a;
	pthread_mutex_unlock(&brk_time_no_maxon_mutex);
};


double c_atlasmv::get_brk_time_no_maxon()
{
	pthread_mutex_lock(&brk_time_no_maxon_mutex);
	double *a = &brk_time_no_maxon;
	pthread_mutex_unlock(&brk_time_no_maxon_mutex);
	return *a;
};
/** 
 * @brief method to allow multitread and get debug flag
 * 
 * @return char with debug flag 
 */
char c_atlasmv::get_executionflags_debug()
{
	pthread_mutex_lock(&flags_mutex);
	char a = debug_mode;
	pthread_mutex_unlock(&flags_mutex);
	return a;
};
/** 
 * @brief method to get errors values
 * 
 * @return integer with errors
 */
int c_atlasmv::get_errors()
{
	pthread_mutex_lock(&errors_mutex);
	int a = errors;
	pthread_mutex_unlock(&errors_mutex);
	return a;
};

/** 
 * @brief method to set errors values
 * 
 * @param err - error to set
 * 
 * @return 
 */
int c_atlasmv::set_errors(int err)
{
	pthread_mutex_lock(&errors_mutex);
	errors = err;
	pthread_mutex_unlock(&errors_mutex);
	return 1;
};

/** 
 * @brief method to  set DES com path
 * 
 * @param com - string with com path
 */
void c_atlasmv::set_com_des(std::string com)
{
	pthread_mutex_lock(&com_des_mutex);
	//sprintf(com_des, "%s", com);
	com_des = com;
	pthread_mutex_unlock(&com_des_mutex);
};

/** 
 * @brief method to get DES com path
 * 
 * @return com path
 */
std::string c_atlasmv::get_com_des()
{
	std::string local;
	pthread_mutex_lock(&com_des_mutex);
	local=com_des;
	//sprintf(a,"%s",com_des);
	pthread_mutex_unlock(&com_des_mutex);
	return local;
};

/** 
 * @brief method to set DES porthandler
 * 
 * @param port - value to define
 */
void c_atlasmv::set_porthandler_des(int port)
{
	pthread_mutex_lock(&porthandler_des_mutex);
	*porthandler_des = port;
	pthread_mutex_unlock(&porthandler_des_mutex);
};

/** 
 * @brief method to get DES porthandler
 * 
 * @return - porthandler
 */
int c_atlasmv::get_porthandler_des()
{
	pthread_mutex_lock(&porthandler_des_mutex);
	int a = *porthandler_des;
	pthread_mutex_unlock(&porthandler_des_mutex);
	return a;
};

/** 
 * @brief  method to set maxon rpm
 * 
 * @param act_rpm - actual motor rpm
 * @param rpm_req - rpm requested to engine
 */
void c_atlasmv::set_rpm(int *act_rpm, int *rpm_req)
{
	pthread_mutex_lock(&rpm_mutex);
	actual_rpm = *act_rpm;
	req_rpm = *rpm_req;
	pthread_mutex_unlock(&rpm_mutex);
};


/** 
 * @brief method to get MAXON rpm
 * 
 * @param rpm - actual engine rpm 
 * @param demanded_rpm - engine rpm demanded
 */
void c_atlasmv::get_rpm(int *rpm, int *demanded_rpm)
{
	pthread_mutex_lock(&rpm_mutex);
	*rpm = actual_rpm;
	*demanded_rpm = req_rpm; 
	pthread_mutex_unlock(&rpm_mutex);
};

/** 
 * @brief method to set back wheel diameter
 * 
 * @param diam - diameter
 */
void c_atlasmv::set_back_wheel_diam(double *diam)
{
	pthread_mutex_lock(&back_wheel_diam_mutex);
	back_wheel_diam = *diam;
	pthread_mutex_unlock(&back_wheel_diam_mutex);
};

/** 
 * @brief method to get robot back wheels diameter
 * 
 * @return back wheel diameter
 */
double c_atlasmv::get_back_wheel_diam()
{
	pthread_mutex_lock(&back_wheel_diam_mutex);
	double a=back_wheel_diam;
	pthread_mutex_unlock(&back_wheel_diam_mutex);
	return a;
};

/** 
 * @brief set MAXON transmission relation
 * 
 * @param i - transmission relation
 */
void c_atlasmv::set_transmission_relation(double *i)
{
	pthread_mutex_lock(&transmission_relation_mutex);
	transmission_relation = *i;
	pthread_mutex_unlock(&transmission_relation_mutex);
};

/** 
 * @brief method to get transmission relation value
 * 
 * @return transmission relation
 */
double c_atlasmv::get_transmission_relation()
{
	pthread_mutex_lock(&transmission_relation_mutex);
	double a=transmission_relation;
	pthread_mutex_unlock(&transmission_relation_mutex);
	return a;
};

/** 
* @brief method to get the linear the robot linear speed
* 
 * @param speed - instantaneous linear velocity
 * @param demanded_speed - requested linear velocity
 */
void c_atlasmv::get_linear_speed(double *speed, double *demanded_speed)
{

	int rpm1,rpm2;
	get_rpm(&rpm1, &rpm2);


	pthread_mutex_lock(&linear_speed_mutex);
	actual_speed = -1*(M_PI*rpm1*get_back_wheel_diam())/(60*get_transmission_relation());
	req_speed = -1*(M_PI*rpm2*get_back_wheel_diam())/(60*get_transmission_relation());

// 	printf("rpm1 = %d\n",rpm1);
// 	printf("actual speed=%f\n",actual_speed);
	if(speed)
	{
		*speed = actual_speed;
	}
	if(demanded_speed)
	{
		*demanded_speed = req_speed;
	}
	pthread_mutex_unlock(&linear_speed_mutex);
}

/** 
 * @brief method to set max forward speed
 * 
 * @param a - value to define
 */
void c_atlasmv::set_max_forward_speed(double *a)
{
	pthread_mutex_lock(&max_forward_speed_mutex);
	max_forward_speed = *a;
	pthread_mutex_unlock(&max_forward_speed_mutex);
};

/** 
 * @brief method to get max forward speed
 * 
 * @return double  with max forward speed
 */
double c_atlasmv::get_max_forward_speed()
{
	pthread_mutex_lock(&max_forward_speed_mutex);
	double a = max_forward_speed;
	pthread_mutex_unlock(&max_forward_speed_mutex);
	return a;
};

void c_atlasmv::set_max_backward_speed(double *a)
{
	pthread_mutex_lock(&max_backward_speed_mutex);
	max_backward_speed = *a;
	pthread_mutex_unlock(&max_backward_speed_mutex);
};

double c_atlasmv::get_max_backward_speed()
{
	pthread_mutex_lock(&max_backward_speed_mutex);
	double a = max_backward_speed;
	pthread_mutex_unlock(&max_backward_speed_mutex);
	return a;
};

void c_atlasmv::set_brake_speed_dif(double *a)
{
	pthread_mutex_lock(&brake_speed_dif_mutex);
	brake_speed_dif = *a;
	pthread_mutex_unlock(&brake_speed_dif_mutex);
};
double c_atlasmv::get_brake_speed_dif()
{
	pthread_mutex_lock(&brake_speed_dif_mutex);
	double a = brake_speed_dif;
	pthread_mutex_unlock(&brake_speed_dif_mutex);
	return a;
};

void c_atlasmv::set_length(double *a)
{
	pthread_mutex_lock(&length_mutex);
	length = *a;
	pthread_mutex_unlock(&length_mutex);
};

double c_atlasmv::get_length()
{
	pthread_mutex_lock(&length_mutex);
	double a = length;
	pthread_mutex_unlock(&length_mutex);
	return a;
};

void c_atlasmv::set_width(double *a)
{
	pthread_mutex_lock(&width_mutex);
	width = *a;
	pthread_mutex_unlock(&width_mutex);
};

double c_atlasmv::get_width()
{
	pthread_mutex_lock(&width_mutex);
	double a = width;
	pthread_mutex_unlock(&width_mutex);
	return a;
};

void c_atlasmv::set_wheelaxisdistance(double *a)
{
	pthread_mutex_lock(&wheelaxisdistance_mutex);
	wheelaxisdistance = *a;
	pthread_mutex_unlock(&wheelaxisdistance_mutex);
};

double c_atlasmv::get_wheelaxisdistance()
{
	pthread_mutex_lock(&wheelaxisdistance_mutex);
	double a = wheelaxisdistance;
	pthread_mutex_unlock(&wheelaxisdistance_mutex);
	return a;
};

void c_atlasmv::set_min_pulse_brk(int *a)
{
	pthread_mutex_lock(&min_pulse_brk_mutex);
	min_pulse_brk = *a;
	pthread_mutex_unlock(&min_pulse_brk_mutex);
};

int c_atlasmv::get_min_pulse_brk()
{
	pthread_mutex_lock(&min_pulse_brk_mutex);
	int a = min_pulse_brk;
	pthread_mutex_unlock(&min_pulse_brk_mutex);
	return a;
};


void c_atlasmv::set_max_pulse_brk(int *a)
{
	pthread_mutex_lock(&max_pulse_brk_mutex);
	max_pulse_brk = *a;
	pthread_mutex_unlock(&max_pulse_brk_mutex);
};

int c_atlasmv::get_max_pulse_brk()
{
	pthread_mutex_lock(&max_pulse_brk_mutex);
	int a = max_pulse_brk;
	pthread_mutex_unlock(&max_pulse_brk_mutex);
	return a;
};

void c_atlasmv::set_min_pulse_dir(int *q)
{
	pthread_mutex_lock(&min_pulse_dir_mutex);
	min_pulse_dir = *q;
	pthread_mutex_unlock(&min_pulse_dir_mutex);
};

int c_atlasmv::get_min_pulse_dir()
{
	pthread_mutex_lock(&min_pulse_dir_mutex);
	int a = min_pulse_dir;
	pthread_mutex_unlock(&min_pulse_dir_mutex);
	return a;
};


void c_atlasmv::set_max_pulse_dir(int *a)
{
	pthread_mutex_lock(&max_pulse_dir_mutex);
	max_pulse_dir = *a;
	pthread_mutex_unlock(&max_pulse_dir_mutex);
};

int c_atlasmv::get_max_pulse_dir()
{
	pthread_mutex_lock(&max_pulse_dir_mutex);
	int a = max_pulse_dir;
	pthread_mutex_unlock(&max_pulse_dir_mutex);
	return a;
};

/** 
 * @brief method to set des configuration
 * 
 * @param sDESconfig - structure according libdes70_10.
 */
void c_atlasmv::set_sDES_sysparam(TYPE_des_sysparam* sDESconfig)
{
	pthread_mutex_lock(&sDES_sysparam_mutex);
	sDES_sysparam = *sDESconfig;
	pthread_mutex_unlock(&sDES_sysparam_mutex);
};

void c_atlasmv::get_sDES_sysparam(TYPE_des_sysparam *b)
{
	pthread_mutex_lock(&sDES_sysparam_mutex);
	*b = sDES_sysparam;
	pthread_mutex_unlock(&sDES_sysparam_mutex);
};

//void c_atlasmv::set_actual_rpm(int *a)
//{
	//pthread_mutex_lock(&actual_rpm_mutex);
	//actual_rpm = sDES_sysparam;
	//pthread_mutex_lock(&sDES_sysparam_mutex);
//};
		//void get_actual_rpm(int *);
		//void set_req_rpm(int *);
		//void get_req_rpm(int *);
		//void set_actual_speed(double *);
		//void get_actual_speed(double *);
		//void set_req_speed(double *);
		//void get_req_speed(double *);

void c_atlasmv::set_des_initialized(char a)
{
	pthread_mutex_lock(&des_initialized_mutex);
	des_initialized=a;
	pthread_mutex_unlock(&des_initialized_mutex);
};

char c_atlasmv::get_des_initialized()
{
	pthread_mutex_lock(&des_initialized_mutex);
	char a=des_initialized;
	pthread_mutex_unlock(&des_initialized_mutex);
	return a;
};
void c_atlasmv::set_com_dioc(std::string a)
{
	pthread_mutex_lock(&com_dioc_mutex);
	com_dioc=a;
	//strcpy(com_dioc, a);
	pthread_mutex_unlock(&com_dioc_mutex);
};

std::string c_atlasmv::get_com_dioc()
{
	std::string local;
	pthread_mutex_lock(&com_dioc_mutex);//ver se isto não é um problema
	local = com_dioc;
	//strcpy(a, com_dioc);
	pthread_mutex_unlock(&com_dioc_mutex);
	return local;
};

void c_atlasmv::set_com_servo(std::string com)
{
	pthread_mutex_lock(&com_servos_mutex);
	com_servos=com;
	pthread_mutex_unlock(&com_servos_mutex);
};//Y

std::string c_atlasmv::get_com_servo()
{
	std::string local;
	pthread_mutex_lock(&com_servos_mutex);
	//printf("aaa:%s\n", com);
	//printf("%s\n", com_servos);
	//a=(char*)malloc(255*sizeof(char));
	//strcpy(a, com_servos);
	local=com_servos;
	pthread_mutex_unlock(&com_servos_mutex);
	return local;
};

void c_atlasmv::set_dir_srv_id(unsigned char a)
{
	pthread_mutex_lock(&servo_direction_id_mutex);
	servo_direction_id = a;
	pthread_mutex_unlock(&servo_direction_id_mutex);
};

char c_atlasmv::get_dir_srv_id()
{
	pthread_mutex_lock(&servo_direction_id_mutex);
	unsigned char a = servo_direction_id;
	pthread_mutex_unlock(&servo_direction_id_mutex);
	return a;
};

void c_atlasmv::set_brk_srv_id(unsigned char a)
{
	pthread_mutex_lock(&servo_brake_id_mutex);
	servo_brake_id = a;
	pthread_mutex_unlock(&servo_brake_id_mutex);
};

char c_atlasmv::get_brk_srv_id()
{
	pthread_mutex_lock(&servo_brake_id_mutex);
	unsigned char a = servo_brake_id;
	pthread_mutex_unlock(&servo_brake_id_mutex);
	return a;
};

void c_atlasmv::set_maximum_brk_angle(double *a)
{
	pthread_mutex_lock(&maximum_brk_mutex);
	maximum_brk = *a;
	pthread_mutex_unlock(&maximum_brk_mutex);
};
double c_atlasmv::get_maximum_brk_angle()
{
	pthread_mutex_lock(&maximum_brk_mutex);
	double a=maximum_brk;
	pthread_mutex_unlock(&maximum_brk_mutex);
	return a;
};

void c_atlasmv::set_minimum_brk_angle(double *a)
{
	pthread_mutex_lock(&minimum_brk_mutex);
	minimum_brk = *a;
	pthread_mutex_unlock(&minimum_brk_mutex);
};
double c_atlasmv::get_minimum_brk_angle()
{
	pthread_mutex_lock(&minimum_brk_mutex);
	double a=minimum_brk;
	pthread_mutex_unlock(&minimum_brk_mutex);
	return a;
};

void c_atlasmv::set_maximum_dir_angle(double *a)
{
	pthread_mutex_lock(&maximum_dir_mutex);
	maximum_dir=*a;
	pthread_mutex_unlock(&maximum_dir_mutex);
};
double c_atlasmv::get_maximum_dir_angle()
{
	pthread_mutex_lock(&maximum_dir_mutex);
	double a=maximum_dir;
	pthread_mutex_unlock(&maximum_dir_mutex);
	return a;
};

void c_atlasmv::set_minimum_dir_angle(double *a)
{
	pthread_mutex_lock(&minimum_dir_mutex);
	minimum_dir=*a;
	pthread_mutex_unlock(&minimum_dir_mutex);
};
double c_atlasmv::get_minimum_dir_angle()
{
	pthread_mutex_lock(&minimum_dir_mutex);
	double a=minimum_dir;
	pthread_mutex_unlock(&minimum_dir_mutex);
	return a;
};


/** 
 * @brief method that initializes and set up the conection with servoamplifier DES
 *
 * @return (0) if some error occurs to open port, (1) success
 */
int c_atlasmv::init_com_des()
{
	int result = 0;

	printf("###\t\33[1m\33[34mSet Up ServoAmplifier\33[0m\t###\n");
	//if(robot_details)
	{
		//printf("com_des: %s\n", com_des);
		int port=1234;
		if(InitDES_communication(get_com_des(), &port))
		{
			//set_des_initialized(1);
			//printf("1-port:%d\n", port);
			set_porthandler_des(port);
			result = restart_des();

		}else{
			perror("Error opening DES serial connection");
		}
	}////else
	//{
		//printf("\33[1m\33[31mError\33[0m not defined comm device path and/or configuration for des 70/10\n");
	//}
		//exit(0);

	printf("###\t\33[34mFinished\33[0m \t###\n");
	//printf("RESULT:%d\n", result);
	return result;
}

/** 
 * @brief method to initialize and set up the communication with servo interface
 * 
 * @return (0) if some error occurs to define and open port, (1) success
 */
int c_atlasmv::init_com_servo()
{
	int result=1;

	printf("###\t\33[1m\33[34mSet Up Servos\33[0m\t###\n");

	servo = new hitec_5980SG(com_servos.c_str());



	unsigned short position=0;
	unsigned short brake=0;
	servo->SetPosition(get_dir_srv_id(), 1500);
	//servo->SetPosition(get_brk_srv_id(), get_min_pulse_brk());
	position = servo->SetSpeedPosition(get_dir_srv_id(), 255);
	brake = servo->SetSpeedPosition(get_brk_srv_id(), 255);

	//printf("pos:%d brk:%d\n", position, brake);

	if((!position) || (!brake))
	{
		printf("\33[1m\33[31mError\33[0m Direction or Brake Servo did not respond\n");
		result = 0;
	};

	printf("###\t\33[34mFinished\33[0m\t###\n");
	return result;
}



/** 
 * @brief method to set robot details. attention if required to reset robot parameters is required to re-initialize the robot (procedure initialize_robot)
 * 
 * @param p_robot - pointer to robot details struct
 * 
 * @return (1) is always successful this op
 */
int c_atlasmv::set_atlasmv_details(TYPE_atlasmv_public_params* p_robot)
{

	pthread_mutex_lock(&c_atlasmv_robot_details_mutex);
	robot_details = p_robot;
	set_max_forward_speed( &p_robot->max_forward_speed);
	set_max_backward_speed( &p_robot->max_backward_speed);
	set_brake_speed_dif( &p_robot->brake_speed_dif);
	set_dir_srv_id( p_robot->servo_direction_id);
	set_brk_srv_id( p_robot->servo_brake_id);
	//device paths
	printf("servo:%s\n", p_robot->com_servos.c_str());
	set_com_servo(p_robot->com_servos.c_str());
	set_com_dioc( p_robot->com_dioc.c_str());
	set_com_des( p_robot->com_des.c_str());
	//phisical aspects
	set_length( &p_robot->length);
	set_width( &p_robot->width);
	set_back_wheel_diam( &p_robot->back_wheel_diam);
	set_wheelaxisdistance( &p_robot->wheelaxisdistance);
	set_maximum_dir_angle( &p_robot->maximum_dir);
	set_minimum_dir_angle( &p_robot->minimum_dir);
	set_maximum_brk_angle( &p_robot->maximum_brk);
	set_minimum_brk_angle( &p_robot->minimum_brk);
	set_transmission_relation( &p_robot->transmission_relation);
	set_min_pulse_brk( &p_robot->min_pulse_brk);
	set_max_pulse_brk( &p_robot->max_pulse_brk);
	set_min_pulse_dir( &p_robot->min_pulse_dir);
	set_max_pulse_dir( &p_robot->max_pulse_dir);
	set_sDES_sysparam( &p_robot->sDES_sysparam);
	set_brk_time_no_maxon( &p_robot->brk_time_no_maxon);
	pthread_mutex_unlock(&c_atlasmv_robot_details_mutex);

	return 1;
};

void c_atlasmv::set_dioc_initialized(char a)
{
	pthread_mutex_lock(&dioc_initialized_mutex);
	dioc_initialized=a;
	pthread_mutex_unlock(&dioc_initialized_mutex);
};
char c_atlasmv::get_dioc_initialized()
{
	pthread_mutex_lock(&dioc_initialized_mutex);
	char a=dioc_initialized;
	pthread_mutex_unlock(&dioc_initialized_mutex);
	return a;
};

void c_atlasmv::set_servo_initialized(char a)
{
	pthread_mutex_lock(&servo_initialized_mutex);
	servo_initialized=a;
	pthread_mutex_unlock(&servo_initialized_mutex);
};


char c_atlasmv::get_servo_initialized()
{
	pthread_mutex_lock(&servo_initialized_mutex);
	char a=servo_initialized;
	pthread_mutex_unlock(&servo_initialized_mutex);
	return a;
};
/** 
 * @brief method to begin communication with DIOC board and check if is connected
 * 
 * @return (0) if DIOC does not respond (1) without errors
 */
int c_atlasmv::init_com_dioc()
{
	int result=1;


	if(!get_com_dioc().empty())
	{
		printf("###\t\33[1m\33[34mSet Up DIOC\33[0m\t###\n");
		dioc = new class_dioc(get_com_dioc().c_str());
	}else
	{
		printf("###\t\33[1m\33[34mDo not init DIOC\33[0m\t###\n");
		dioc = new class_dioc(get_com_dioc().c_str());
	}
	



	//printf("dioc result:%d", result);
	if(dioc->CommStatus()==-1)
	{
		result = 0;
		printf("\33[1m\33[31mError\33[0m DIOC did not respond\n");
	}


	printf("###\t\33[34mFinished\33[0m\t###\n");
	
	return result;
};

/** 
 * @brief method that initialize interfaces with lower level devices
 * 
 * @return (0) some error occurs when opening low level connections, (1) success
 */
int c_atlasmv::initialize_robot(void)
{
	int result = 1;
	int a,b,c;
	
	if(init_com_dioc())
	{
		set_dioc_initialized(1);
		a=0;
	}

	if(init_com_des())
	{
		set_des_initialized(1);
		b=0;
	}

	if(init_com_servo())
	{
		set_servo_initialized(1);
		c=0;
	}

	//printf("a:%d b:%d c:%d\n",a,b,c);
	result = (((a)<<1) | ((b) << 2) | ((c)<<3));

	if(!result)
		result = 1;

	timer->tic(0);
	return result;
};


/** 
 * @brief method to start or restart des according config previously defined with set_des_sparam method
 * 
 * @return (0) - 
 */
int c_atlasmv::restart_des()
{
	TYPE_des_sysparam a;
	get_sDES_sysparam(&a);
	//pthread_mutex_unlock(&c_atlasmv_robot_details_mutex);

// 	printf("baud:%d sys_param:%b\n", a.baudrate, a.sys_config);
	sleep(1);
	int port = get_porthandler_des();
	//printf("handler_des:%d\n", get_porthandler_des());
	tcflush(port, TCIOFLUSH);
	if(!InitDES(port, &a, 1, 1, get_executionflags_debug()))
		{
			tcflush(port, TCIOFLUSH);
			set_errors(ENUM_ERR_SUPPLY_VOLT_LOW);
			//errors = ;
			//printf("\33[1m\33[31mError\33[0m des 70/10 did not respond to configuration\n");
			return 0;
		}

	return 1;
}

/** 
 * @brief private method to verify if there exist any issues with des servo amplifier
 * 
 * @return ( 
 */
int c_atlasmv::check_des(void)
{
	int result = 1;
	int error;
	int des_status;

	int port=get_porthandler_des();
	char c=get_executionflags_debug();
	result = DES_ST_read_sys_status(port, &des_status, c);

	bool done_A=false,done_B=false;

	if(des_status & 0b0000001000000000)
	{
		done_A=true;
		tcflush(port, TCIOFLUSH);
		result = DES_ST_read_error(port, &error, c);
		if (result)
		{
			tcflush(port, TCIOFLUSH);
			result = DES_ST_clear_errors(port, c);
			printf("Clear errors\n");
		}
		printf("Result of clear erros %d\n",result);
	}

	if((des_status & 0b0010000000000000) || !(des_status & 0b0000010000000000))
	{
		done_B=true;
		tcflush(port, TCIOFLUSH);
		printf("Restarting des\n");
		set_des_initialized(restart_des());
	}
	
	if(!done_A && !done_B)
	{
		tcflush(port, TCIOFLUSH);
		printf("Forcing des restart\n");
		set_des_initialized(restart_des());
	}
	
	return result;
};




/** 
 * @brief method to read from servoamplifier the speed
 * 
 * @return (0) not possible to read speed, (1) speed readed
 */
int c_atlasmv::read_speed(int speed_type)
{
	int result = 0;
	static int rpm1=0,rpm2=0;
	//Added by mike and joel to solve the problem of having speed = 0 when there is a problem comunicating with DES
	int port=get_porthandler_des();
	char debug = get_executionflags_debug();

	if(!get_des_initialized())
	{
		//rpm1=0; rpm2=0;
		set_rpm(&rpm1, &rpm2);
		return result;
	}

	
	tcflush(port, TCIOFLUSH);
	if(DES_MF_read_velocity_is_must(port, speed_type, &rpm1, &rpm2, debug))
	{
		set_rpm(&rpm1, &rpm2);
		result = 1;
	}else
	{
		//rpm1=0; rpm2=0;
		set_rpm(&rpm1, &rpm2);
		set_des_initialized(0);
		printf("cannot com with DES\n");
	};

	return result;
};



/** 
 * @brief method to set the robot speed
 * 
 * @param newVel - value in m/s
 * 
 * @return (0) error - velocity not sent (1) everything ok
 */
int c_atlasmv::set_speed(double *newVel)
{
	int result = 1;
	

	int rpm = -1*round((60*(get_transmission_relation())*(*newVel))/(M_PI*(get_back_wheel_diam())));
	//new_rpm = -1*cvRound((60*(*newVel))/((robot_details->back_wheel_diam/2.0)*robot_details->transmission_relation));

	//to garantee that the rpm is under the limits
	TYPE_des_sysparam a;
 	get_sDES_sysparam(&a);
	((fabs(rpm) > a.max_speed)? rpm = (rpm>0?1:-1)*a.max_speed: 0);
// 	rpm = 1000;


	if(get_des_initialized())
	{
		int port = get_porthandler_des();
		char a = get_executionflags_debug();
		result = DES_SF_set_velocity(port, rpm, a);
		if(!result)
			set_des_initialized(0);
	}
	else
		result=0;

	fflush(stdout);

	//printf("speed set:%3.2f|%d\n", *newVel, rpm);
	return result;
};


/** 
 * @brief method to set the robot direction
 * 
 * @param ang - value in radians
 * 
 * @return (0) error - velocity not sent (1) everything ok
 */
int c_atlasmv::set_direction(double *ang)
{
	cout<<"ANG: "<<*ang<<endl;
	int result = 1;
	
	*ang=*ang>get_maximum_dir_angle()?get_maximum_dir_angle():*ang;
	*ang=*ang<get_minimum_dir_angle()?get_minimum_dir_angle():*ang;
	
	int numpulses = rad2pulses(*ang);
	
	if(numpulses > get_max_pulse_dir())
			numpulses = get_max_pulse_dir();
	
	if(numpulses < get_min_pulse_dir())
		numpulses = get_min_pulse_dir();

	int a = get_dir_srv_id();
	if(get_servo_initialized())
	{
		printf("Id %d *ang:%2.2f pulses:%d\n",a, *ang, numpulses);
		servo->SetPosition(a, numpulses);
		servo->SetSpeedPosition(a, 255);
	}else{
		result=0;
	};

	return result;
};


/** 
 * @brief method to request actual direction
 * 
 * @param ang - value in radians
 * 
 * @return (0) error (1) everything ok
 */
int c_atlasmv::read_direction(double *ang)
{
	int result = 1;

	if(!get_servo_initialized())
		return 0;

	int a = get_dir_srv_id();
		//robot_details->servo_direction_id;

	unsigned short position = servo->SetSpeedPosition(a, 255);

	if(position==0xFFFF)
	{
		result = 0;
		set_errors(ENUM_ERR_READ_DIR);
	}
	else
	{
		double a = pulses2rad(position);
		cout<<"Pulses Read: "<<position<<" angle: "<<a<< endl;
		
		set_actual_dir(&a);
		*ang=a;
// 		read_direction(ang);
	}
// 	printf("ang:%.3f pos %d\n", *ang,position);
	return result;
};


/** 
 * @brief method to convert from pulse to radians for the direction servo
 * 
 * @param pulses - servo position in pulses
 * 
 * @return calculated angle
 */
double c_atlasmv::pulses2rad(int pulses)
{
	static bool init=true;
	static vector<pair<double,double> > calib_pul2rad;
	
	if(init)
	{
		calib_pul2rad.push_back(make_pair(900,0.4283));
		calib_pul2rad.push_back(make_pair(1500,-0.009315));
// 		calib_pul2rad.push_back(make_pair(2100,-0.5087));
		calib_pul2rad.push_back(make_pair(2100,-0.55));
		
		init=false;
	}
	return map_linear((double)pulses,calib_pul2rad);
	
}

/** 
 * @brief method to convert from radians to pulses
 * 
 * @param rad - angle in radians
 * 
 * @return number of pulses
 */
int c_atlasmv::rad2pulses(double rad)
{
	static bool init=true;
	static vector<pair<double,double> > calib_rad2pul;
	
	if(init)
	{
// 		calib_rad2pul.push_back(make_pair(-0.5087,2100));
		calib_rad2pul.push_back(make_pair(-0.55,2100));
		calib_rad2pul.push_back(make_pair(-0.009315,1500));
		calib_rad2pul.push_back(make_pair(0.4283,900));
		
		init=false;
	}
	
	return round(map_linear(rad,calib_rad2pul));
	
}


/** 
 * @brief method to read the brake servo position 
 * 
 * @param brk servo position
 * @param num_pulses variable to store the number of pulses
 * @return (0) error (1) everything ok
 */
int c_atlasmv::read_brake(double *brk, int *num_pulses)
{
	int result = 1;

	double m = ((get_max_pulse_brk()-get_min_pulse_brk())/(get_maximum_brk_angle()-get_minimum_brk_angle()));

	if(!get_servo_initialized())
		return 0;

	unsigned short numpulses = servo->SetSpeedPosition(get_brk_srv_id(), 255);

	*brk=(numpulses-get_min_pulse_brk())/m;

	//*brk=(double)(numpulses-get_max_pulse_brk());
	set_actual_brk(brk);
	if(num_pulses)
	{
		*num_pulses = numpulses;
	}

	//printf("numpulses:%d, ang:%2.2f\n", numpulses, *brk);

	return result;

};


/** 
 * @brief method to send servo order to brake
 * 
 * @param brk - brake angle
 * 
 * @return - (0) not braking, (1) success 
 */
int c_atlasmv::set_brake(double *brk)
{
	int result = 1;

	int a = get_max_pulse_brk();
	int b = get_min_pulse_brk();

	unsigned short numpulses = round((*brk)*(a-b)+b);

	if(numpulses > a)
			numpulses = a;
	else if(numpulses < b)
		numpulses = b;

	if(get_servo_initialized())
	{
		servo->SetPosition(get_brk_srv_id(), numpulses);
		servo->SetSpeedPosition(get_brk_srv_id(), 255);
		//printf("set:numpulses:%d, ang:%2.2f\n", numpulses, *brk);
	}else
		result=0;

	return result;
};

/** 
 * @brief method to set a new robot state.
 * 
 * @param sign_command - command that says what sign was detected. Robot Magic Ball :)
 * @param lights_command - command that says if is required to turn lights on (turn light, front and/or back)
 * @param motion_command - command with new robot state
 * 
 * @return - (0)  (1) everything set without issue
 */
int c_atlasmv::new_robot_state(atlasmv_base::AtlasmvMotionCommand&motion_command,atlasmv_base::AtlasmvVertSignsCommand&sign_command,atlasmv_base::AtlasmvLightsCommand&lights_command)
{
	#define DES_VELOCITY_TYPE 1 //velocity can be 0 - mean value or 1 - instant
	int result = 0;
	//double r_speed;
	double new_brake = 0;
	

	if(!get_des_initialized() && !get_dioc_initialized() && !get_servo_initialized())
		return result;

	set_errors(ENUM_ERR_NONE);

	timer->toc(0);
	if(timer->get_toc(0)>0.5)
	{
		check_des();
		timer->tic(0);
	}
		

	//set new speed and verify if is required to brake
	//thread1
	read_speed(DES_VELOCITY_TYPE);//
	double speed_mom, speed_demanded;
	get_linear_speed(&speed_mom, &speed_demanded);

	//thread2
	double direction;
	read_direction(&direction);

	//truncating speed
	motion_command.speed=motion_command.speed>get_max_forward_speed()?get_max_forward_speed():motion_command.speed;
	motion_command.speed=motion_command.speed<get_max_backward_speed()?get_max_backward_speed():motion_command.speed;

	if(fabs((motion_command.speed)) < fabs(speed_mom))
	{
		//thread3
		set_speed(&motion_command.speed);

		if(fabs((motion_command.speed)- speed_mom)>get_brake_speed_dif())
		{
			new_brake = 1.0;
		}else
		{
			new_brake = 0.0;
		};

// 		if(debug_mode)printf("new_speed:%.3f, actual_speed:%.3f brk:%.3f\n", motion_command.speed, speed_mom, new_brake);
		set_brake(&new_brake);
	}else
	{
		new_brake = 0.0;
		set_brake(&new_brake);
		//DES_ST_enable(*porthandler_des, &des_enable, 0);
		set_speed(&motion_command.speed);
		if(debug_mode)printf("new_speed:%.3f, actual_speed:%.3f\n", motion_command.speed, speed_mom/*, fabs(motion_command.speed - actual_speed)*/);
	
	};
	

	
	if(fabs(direction - (motion_command.dir)) > 0.0349)
	{
		//set_direction(&(motion_command.dir));
	}
	else
	{
		//set_direction(&direction);
	}

			
	set_lights(sign_command, lights_command);

			
	return result;
};


/** 
 * @brief method to set lights new state
 * 
 * @param sign_command - command with vertical sign state (mandatory, warning, information)
 * @param lights_command - command with new state for the general lights (left, right, head, tail & reverse)
 * 
 * @return 
 */
int c_atlasmv::set_lights(atlasmv_base::AtlasmvVertSignsCommand&sign_command,atlasmv_base::AtlasmvLightsCommand&lights_command)
{
	int result = 1;

	pthread_mutex_lock(&robot_status_mutex);
	pthread_mutex_lock(&sign_mutex);
	//printf("class:vert:%d, lights{%d,%d,%d,%d,%d}\n",sign_command.vert_sign, lights_command.reverselights, lights_command.taillights, lights_command.headlights, lights_command.turnleft, lights_command.turnright);
	if(sign_command.vert_sign != prev_sign)
	{
		prev_sign = sign_command.vert_sign;
		switch (prev_sign)
		{
			case MANDATORY_MEDIUM_LIGTHS://blue
				dioc->SetStatus(dioc->BLUE, dioc->BLINK1);
				break;
			case MANDATORY_BUS_LANE://blue
				dioc->SetStatus(dioc->BLUE, dioc->BLINK3);
				break;
			case WARNING_DIP_AHEAD://red
				dioc->SetStatus(dioc->RED, dioc->BLINK1);
				break;
			case WARNING_ROAD_NARROWS://red
				dioc->SetStatus(dioc->RED, dioc->BLINK3);
				break;
			case INFORMATION_HOSPITAL://green
				dioc->SetStatus(dioc->GREEN, dioc->BLINK1);
				break;
			case INFORMATION_REC_SPEED_60://green
				dioc->SetStatus(dioc->GREEN, dioc->BLINK3);
				break;

		}
	};
	vert_sign = sign_command.vert_sign;
	pthread_mutex_unlock(&sign_mutex);


	pthread_mutex_lock(&lights_mutex);

	int a = lights_command.headlights+lights_command.reverselights+lights_command.taillights+lights_command.turnleft+lights_command.turnright;
	if(a != prev_lights)
	{
		//printf("a %d\n",a);
		prev_lights = a;
		dioc->SetStatus(dioc->LEFT, lights_command.turnleft);
		turnleft = lights_command.turnleft;
		dioc->SetStatus(dioc->RIGHT, lights_command.turnright);
		turnright = lights_command.turnright;

		dioc->SetStatus(dioc->FRONT, lights_command.headlights);
		headlights = lights_command.headlights;
		dioc->SetStatus(dioc->REVERSE, lights_command.reverselights);
		reverselights = lights_command.reverselights;
		dioc->SetStatus(dioc->BRAKE, lights_command.taillights);
		taillights = lights_command.taillights;
	}

	pthread_mutex_unlock(&lights_mutex);
	pthread_mutex_unlock(&robot_status_mutex);
	return result;
};

void c_atlasmv::set_cross_sensor(int a)
{
	pthread_mutex_lock(&cross_sensor_mutex);
	cross_sensor=a;
	pthread_mutex_unlock(&cross_sensor_mutex);
};


int c_atlasmv::get_cross_sensor()
{
	pthread_mutex_lock(&cross_sensor_mutex);
	int a = cross_sensor;
	pthread_mutex_unlock(&cross_sensor_mutex);
	return a;
};

/** 
 * @brief method to get crosswalk sensors
 * 
 * @return (1) sensor detected (0) not detected
 */
int c_atlasmv::read_cross_sensor()
{
	int a=(dioc->GetStatus(dioc->CROSS_A));
	set_cross_sensor(a);
	return a;
};

void c_atlasmv::set_actual_dir(double *a)
{
	pthread_mutex_lock(&actual_dir_mutex);
	actual_dir = *a;
	pthread_mutex_unlock(&actual_dir_mutex);
};

double c_atlasmv::get_actual_dir()
{
	pthread_mutex_lock(&actual_dir_mutex);
	double a=actual_dir;
	pthread_mutex_unlock(&actual_dir_mutex);
	return a;
};

void c_atlasmv::set_actual_brk(double *a)
{
	pthread_mutex_lock(&actual_brk_mutex);
	actual_brk = *a;
	pthread_mutex_unlock(&actual_brk_mutex);
}

double c_atlasmv::get_actual_brk()
{
	pthread_mutex_lock(&actual_brk_mutex);
	double a=actual_brk;
	pthread_mutex_unlock(&actual_brk_mutex);
	return a;
};


/** 
 * @brief used to get lights values
 * 
 * @param robot_status
 * 
 * @return 
 */
int c_atlasmv::get_lights(atlasmv_base::AtlasmvStatus& robot_status)
{
	pthread_mutex_lock(&robot_status_mutex);
	robot_status.headlights = headlights;
	robot_status.reverselights = reverselights;
	robot_status.taillights = taillights;
	robot_status.turnleft = turnleft;
	robot_status.turnright = turnright;
	robot_status.vert_sign = vert_sign ;
	pthread_mutex_unlock(&robot_status_mutex);

	return 1;
};

/** 
 * @brief to update the robot status
 * 
 * @param robot_status - with all relevant fields about robot status
 * 
 * @return is always successful
 */
int c_atlasmv::update_robot_status(atlasmv_base::AtlasmvStatus&robot_status)
{
	int result = 1;
	//double r_speed;

	//read_speed(&actual_speed, &r_speed, DES_VELOCITY_TYPE);

	//read_direction(&actual_dir);

	//robot_status->cross_sensor = read_cross_sensor();

	robot_status.errors = get_errors();
	robot_status.dir = get_actual_dir();
	get_linear_speed(&robot_status.speed, NULL);
	robot_status.cross_sensor = get_cross_sensor();
	get_lights(robot_status);



	//printf("dir:%.4f brk:%.4f speed:%.3f\n", actual_dir, actual_brk, actual_speed);

	return result;
};
