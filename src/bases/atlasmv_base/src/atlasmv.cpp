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
 * @brief Main control module for the atlasmv base
 */

#include <atlasmv_base/atlasmv.h>

#define BRK_TIME 1.0

void handler_vert_sign (const atlasmv_base::AtlasmvVertSignsCommandPtr& msg)
{
	atlasmv_vert_signs=msg;
}

void handler_lights(const atlasmv_base::AtlasmvLightsCommandPtr& msg)
{
	atlasmv_lights=msg;
}

void handler_motion(const atlasmv_base::AtlasmvMotionCommandPtr& msg)
{
// 	printf("Requested Dir: %f (%f degrees)\n",msg->dir,(msg->dir)*180./M_PI);
// 	if(msg->dir>=0.0)
// 		msg->dir=((msg->dir)/1.12)-0.00745;
// 	else
// 		msg->dir=((msg->dir)/1.3)-0.00745;
	
	//check speed limits
	msg->speed=((msg->speed > atlasmv_details.max_forward_speed)?atlasmv_details.max_forward_speed:msg->speed);
	msg->speed=((msg->speed < atlasmv_details.max_backward_speed)?atlasmv_details.max_backward_speed:msg->speed);
	//check direction values
	msg->dir=((msg->dir > atlasmv_details.maximum_dir)?atlasmv_details.maximum_dir:msg->dir);
	msg->dir=((msg->dir < atlasmv_details.minimum_dir)?-atlasmv_details.minimum_dir:msg->dir);

// 	printf("Converted Dir: %f  (%f degrees)\n",msg->dir,(msg->dir)*180./M_PI);
// 	(t->alpha[current_node])/1.12)-0.00745
// 	double bias=0.00745;     //0.0045   
// 		double factor_pos=1.12;     //1.05
// 		double factor_neg=1.3;
// 		
// 		atlasmv_status->dir=atlasmv_status->dir+bias;
// 		
// 		if(atlasmv_status->dir>0)
// 		{
// 			atlasmv_status->dir*=factor_pos;
// 		}
// 		else
// 		{
// 			atlasmv_status->dir*=factor_neg;
// 		}
// 		
// 		atlasmv_status->speed=(atlasmv_status->speed)*1.02;   //1.10
// 	
// 	
	
	command_queue.push_msg(msg);//push new messages into the list
}

/** 
 * @brief thread function that will communicate with DES maxon servoamplifier
 * 
 * @return void
 */
void *des_thread_func(void *)
{
	#define DES_VELOCITY_TYPE 1 //velocity can be 0 - mean value or 1 - instant
	c_timer timer_des;
	timer_des.tic(0);

	double actual_speed;
	double new_speed;
// 	int brake_now=0;
	
	new_speed = 0.00;
	while(1)
	{
		timer_des.tic(9);
		timer_des.toc(0);
		//printf("des_status:%d\n", atlasmv->get_des_initialized());
		if( (!atlasmv->get_des_initialized()) )
		{
			printf("DES:error time:\n");
// 			if(timer_des.get_toc(0)>0.02)
			if(timer_des.get_toc(0)>0.1)
			{
				atlasmv->check_des();
				pthread_mutex_lock(&count_des_errors_mutex);
				count_des_errors++;
				//printf("tried to re-arm maxon servoAmplifier:%d\n", count_des_errors);
				pthread_mutex_unlock(&count_des_errors_mutex);
				timer_des.tic(0);
			}
		}else{
			pthread_mutex_lock(&count_des_errors_mutex);
			count_des_errors=0;
			pthread_mutex_unlock(&count_des_errors_mutex);
		}

		//get new speed turn on flag
		pthread_mutex_lock(&command_mutex);
		
		new_speed = atlasmv_motion->speed;
		cout<<"DES Speed :"<<new_speed<<endl;
		pthread_mutex_unlock(&command_mutex);
		
		//conversion to have velocity is made. turn on flag
		
		timer_des.tic(2);
		atlasmv->read_speed(DES_VELOCITY_TYPE);
		timer_des.toc(2);
		atlasmv->get_linear_speed(&actual_speed, NULL);


		speed_check = 0;
		if(new_speed <= 0.05)	
		{
			pthread_mutex_lock(&speed_check_mutex);
			speed_check = 1;
			pthread_mutex_unlock(&speed_check_mutex);
			pthread_mutex_lock(&brake_check_mutex);
// 			brake_now = brake_check;
			pthread_mutex_unlock(&brake_check_mutex);
// 			if(brake_now)
// 				printf("new speed %3.2f and braking in des tread\n", new_speed);
// 			brake_now=0;
		}
		atlasmv->set_speed(&new_speed);
		
		timer_des.toc(9);
		printf("DES:new-speed:%1.2f actual_speed:%1.2f.\n", new_speed, actual_speed);
		//usleep(1000);//is required a sleep to avoid collisions between threads
		//usleep(15000);
		timer_des.run_sleep(9, 20);//requires this thread to run at a maximal of 30 Hz

	}
	
	return NULL;
};


/** 
 * @brief thread function to interface communication with DIOC board
 * 
 * @return void
 */
void *dioc_thread_func(void *)
{
	c_timer timer_dioc;	

	while(1)
	{
		timer_dioc.tic(9);

		atlasmv->set_lights(*atlasmv_vert_signs,*atlasmv_lights);
		atlasmv->read_cross_sensor();

		timer_dioc.toc(9);
		timer_dioc.run_sleep(9, 10);//requires this thread to run at a maximal of 10 Hz
		//printf("DIOC:sleep:%.5f\n", timer_dioc.get_toc(9));
	}
	
	return NULL;
};

/** 
 * @brief 
 * 
 * @return 
 */
void *servos_thread_func(void *)
{
	double actual_speed, req_speed;
	double new_brake=0;
	double pos0_brake=0.0;
	double new_dir, actual_dir;
	int num_pulses;
	c_timer timer_servos;
	req_speed=0.0;
	bool force_brake=false;
	atlasmv->read_brake(&pos0_brake, &num_pulses);
	atlasmv->set_min_pulse_brk(&num_pulses);
	int speed_now = 0;


	while(1)
	{
		timer_servos.tic(9);//the thread must work with a frequency of 30Hz at maximum
		//direction
		pthread_mutex_lock(&command_mutex);
		
		new_dir = atlasmv_motion->dir;
		cout<<"Dir: "<<atlasmv_motion->dir<<endl;
		req_speed = atlasmv_motion->speed;
		
		pthread_mutex_unlock(&command_mutex);

		atlasmv->get_linear_speed(&actual_speed, NULL);
		//brake analysis
		if( (fabs(req_speed) < fabs(actual_speed)) )
		{
			if( fabs(req_speed-actual_speed) > atlasmv->get_brake_speed_dif() )
			{
				new_brake=1.0;
			}
			else{
				new_brake=0.0;
			}
		}else{
			new_brake=0.0;
		}
		

		// next 3 if to force brake when Maxon ServoAmplifier is not connected
		pthread_mutex_lock(&count_des_errors_mutex);
		int errs = count_des_errors;
		pthread_mutex_unlock(&count_des_errors_mutex);
		//printf("errs:%d actual:%f\n", errs, actual_speed);
		if(force_brake==false && !atlasmv->get_des_initialized() /*&& ( fabs(actual_speed) < 0.1)*/ && (!errs))
		{
			force_brake = true;
			timer_servos.tic(0);
			//previous_speed = actual_speed;
			//printf("SRV: prev:%f act:%f\n", previous_speed, actual_speed);

		}

		//timer_servos.tic(0);
		if( (!atlasmv->get_des_initialized()) && force_brake )
		{
// 			new_brake=1.0;
// 			printf("srv:braking time:%f\n", lar_get_time());
		}

		timer_servos.toc(0);
		if( timer_servos.get_toc(0) > atlasmv->get_brk_time_no_maxon()  && force_brake)
		{
			force_brake = false;
		}

		if(new_brake)	
		{
			pthread_mutex_lock(&speed_check_mutex);
			speed_now = speed_check;
			pthread_mutex_unlock(&speed_check_mutex);
			pthread_mutex_lock(&brake_check_mutex);
			brake_check = 1;
			pthread_mutex_unlock(&brake_check_mutex);
			if(speed_now)
				printf("error 2.0 @ servos_thread\n");
			speed_now=0;
		}
		atlasmv->set_brake(&new_brake);

		atlasmv->read_direction(&actual_dir);
// 		cout<<"Actual dir: "<<actual_dir<<endl;
// 		cout<<"New dir: "<<new_dir<<endl;
// 		printf("DIFERENÇA: %f\n",actual_dir - new_dir);
		
		if( fabs(actual_dir - new_dir) > 0.0349/4.0)//approximatively 0.5º
		{
// 			cout<<"New dir: "<<new_dir<<endl;
			atlasmv->set_direction(&new_dir);
			
		}else{
			
// 			cout<<"Brake dir: "<<actual_dir<<endl;
			atlasmv->set_direction(&actual_dir);
		}
		timer_servos.toc(9);//the thread must work with a frequency of 30Hz at maximum
		//printf("DES:req-speed:%1.2f actual_speed:%1.2f.\n", req_speed, actual_speed);
		//printf("SRV:new-dir:%1.2f actual_dir:%1.2f. freq_actual:%2.0f\n", new_dir, actual_dir, (1.0/timer_servos.get_toc(9)));
		timer_servos.run_sleep(9, 30);
	}
	
	return NULL;
};


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
	
	return out;
}


int main(int argc, char **argv)
{
// 	vector<std::pair<double,double> > calib_pul2rad;
// 	
// 	calib_pul2rad.push_back(make_pair(1005,0.351726));
// 	calib_pul2rad.push_back(make_pair(1500,-0.009315));
// 	calib_pul2rad.push_back(make_pair(1952,0.385515));
//
// 	vector<std::pair<double,double> > calib_rad2pul;
// 	
// 	calib_rad2pul.push_back(make_pair(0.351726,1005));
// 	calib_rad2pul.push_back(make_pair(-0.009315,1500));
// 	calib_rad2pul.push_back(make_pair(0.385515,1952));
// 	
// 	double pul=1200;
// 	
// 	cout<<"In: "<<pul<<" Out: "<<map_linear(pul,calib)<<endl;
// 	
// 	return 0;
	
	atlasmv_help(0);

	atlasmv_details.com_servos = (char*)malloc(256*sizeof(char));
	atlasmv_details.com_des = (char*)malloc(256*sizeof(char));
	atlasmv_details.com_dioc = (char*)malloc(256*sizeof(char));

	atlasmv_details.sleep_time = 1.0/atlasmv_details.cycle_freq;
	
	atlasmv_base::AtlasmvMotionCommandPtr safety_command(new atlasmv_base::AtlasmvMotionCommand);
	
	safety_command->lifetime=INFINITY;
	safety_command->priority=0;	
	safety_command->speed=0;
	safety_command->dir=0;
	
	atlasmv_status.reset(new atlasmv_base::AtlasmvStatus);
	atlasmv_vert_signs.reset(new atlasmv_base::AtlasmvVertSignsCommand);
	atlasmv_lights.reset(new atlasmv_base::AtlasmvLightsCommand);
	
	atlasmv= new c_atlasmv(0);
	
	/**ROS INIT*/
	ros::init(argc, argv, "atlasmv_base");
	ros::NodeHandle nh("~");
	
	command_queue.push_msg(safety_command);
	
	atlasmv_motion=command_queue.top_msg();//get top layer message, just to allow the threads to run
	
	/**Ir buscar os parametros*/
	nh.param("max_forward_speed", atlasmv_details.max_forward_speed,4.0);
	nh.param("max_backward_speed", atlasmv_details.max_backward_speed,-1.);
	nh.param("brake_speed_dif", atlasmv_details.brake_speed_dif,0.25);
	
	nh.param("servo_direction_id", atlasmv_details.servo_direction_id,0);
	nh.param("servo_brake_id", atlasmv_details.servo_brake_id,1);
	
	if (nh.getParam("comport_servos", atlasmv_details.com_servos))
		ROS_INFO("comport_servos: %s", atlasmv_details.com_servos.c_str());
	else
	{
		ROS_ERROR("Failed to get critical param 'comport_servos'");
		return -1;
	}
	
	if (nh.getParam("comport_DES", atlasmv_details.com_des))
		ROS_INFO("comport_DES: %s", atlasmv_details.com_des.c_str());
	else
	{
		ROS_ERROR("Failed to get critical param 'comport_DES'");
		return -1;
	}
	
	if (nh.getParam("comport_dioc", atlasmv_details.com_dioc))
		ROS_INFO("comport_dioc: %s", atlasmv_details.com_dioc.c_str());
	else
	{
		atlasmv_details.com_dioc.clear();
		ROS_WARN("Failed to get param 'comport_dioc'");
	}
	
	nh.param("length", atlasmv_details.length,0.8);
	nh.param("width", atlasmv_details.width,0.6);
	nh.param("wheelaxisdistance", atlasmv_details.wheelaxisdistance,0.49);
	nh.param("back_wheel_diam", atlasmv_details.back_wheel_diam,0.115);
	nh.param("maximum_dir", atlasmv_details.maximum_dir,0.355);
	nh.param("minimum_dir", atlasmv_details.minimum_dir,-0.38);
	nh.param("maximum_brk", atlasmv_details.maximum_brk,1.0);
	nh.param("minimum_brk", atlasmv_details.minimum_brk,0.0);
	nh.param("transmission_relation", atlasmv_details.transmission_relation,13.73);
	nh.param("min_pulse_brk", atlasmv_details.min_pulse_brk,1060);
	nh.param("max_pulse_brk", atlasmv_details.max_pulse_brk,1400);
	nh.param("min_pulse_dir", atlasmv_details.min_pulse_dir,930);
	nh.param("max_pulse_dir", atlasmv_details.max_pulse_dir,2000);
	nh.param("brk_time_no_maxon", atlasmv_details.brk_time_no_maxon,1.0);

	nh.param("des_sys_config", atlasmv_details.sDES_sysparam.sys_config,49024);
	nh.param("des_enc_res", atlasmv_details.sDES_sysparam.enc_resolution,500);
	nh.param("des_max_rpm", atlasmv_details.sDES_sysparam.max_speed,13000);
	nh.param("des_max_cont_current", atlasmv_details.sDES_sysparam.max_cont_current,8000);
	nh.param("des_peak_current", atlasmv_details.sDES_sysparam.peak_current,25000);
	nh.param("des_speed_gain_p", atlasmv_details.sDES_sysparam.speed_reg_gain_p,554);
	nh.param("des_speed_gain_i", atlasmv_details.sDES_sysparam.speed_reg_gain_i,89);
	
	/*Colocar os parameteros que já devem estar em atlasmv_details para dentro da class*/
	initialize_motion_model(&motion_model, 0, 0, 0, atlasmv_details.wheelaxisdistance);	
	
	atlasmv->set_atlasmv_details(&atlasmv_details);
	
	int initialize = atlasmv->initialize_robot();
	if(!(initialize & 1))
	{
		if(initialize & 2)
			printf("\33[31mError with DIOC\33[0m\n");

		if(initialize & 4)
			printf("\33[31mError with ServoAmplifier\33[0m\n");

		if(initialize & 8)
			printf("\33[31mError with Servos\33[0m\n");
	}
	
	des_iret = pthread_create(&des_thread, NULL, des_thread_func , NULL);
	servos_iret = pthread_create(&servos_thread, NULL, servos_thread_func , NULL);
	dioc_iret = pthread_create(&dioc_thread, NULL, dioc_thread_func , NULL);

	/*Substituir por ROS*/
	/*Subscrever as mensagens e usar os handlers que estão lá em cima*/
	ros::Subscriber sub_vert_sign = nh.subscribe("/atlasmv/base/vertical_signs", 1000, handler_vert_sign);
	ros::Subscriber sub_lights = nh.subscribe("/atlasmv/base/lights", 1000, handler_lights);
	ros::Subscriber sub_motion = nh.subscribe("/atlasmv/base/motion", 1000, handler_motion);
	/*Preparar para publicar status*/
	ros::Publisher pub_status=nh.advertise<atlasmv_base::AtlasmvStatus>("/atlasmv/base/status",1000);

	ros::Rate LoopRate(30);
	
	std::cout<<atlasmv_details<<endl;
	
	std::cout<<"Start to spin"<<endl;
	timer.tic(1);
	while (ros::ok())
	{
		atlasmv_motion=command_queue.top_msg();//get top layer message
// 		cout<<*atlasmv_motion<<endl;
		
		atlasmv->update_robot_status(*atlasmv_status);

// 		printf("atlasmv_status->speed=%f\n", atlasmv_status->speed);
		// Calibration
		// first calibrate bias going in a straight line, then calibrate speed and finaly the factors
// 		double bias=0.00745;     //0.0045   
// 		double factor_pos=1.12;     //1.05
// 		double factor_neg=1.3;
// 		
// 		atlasmv_status->dir=atlasmv_status->dir+bias;
// 		
// 		if(atlasmv_status->dir>0)
// 		{
// 			atlasmv_status->dir*=factor_pos;
// 		}
// 		else
// 		{
// 			atlasmv_status->dir*=factor_neg;
// 		}
		
		atlasmv_status->speed=(atlasmv_status->speed)*1.02;   //1.10
		
		update_motion_model(&motion_model, atlasmv_status->speed, atlasmv_status->dir);
		timer.toc(1);
		increment_motion_model(&motion_model, timer.get_toc(1));
		//printf("taxa:%.3f\n", timer.get_toc(1));
		timer.tic(1);

		atlasmv_status->x = motion_model.X;
		atlasmv_status->y = motion_model.Y;
		atlasmv_status->orientation = motion_model.HA;
		atlasmv_status->distance_traveled = motion_model.S;

		//printf("S %f D %f E %d C: %d\n",atlasmv_status->speed,atlasmv_status->dir, atlasmv_status->errors, atlasmv_status->cross_sensor);
		//printf("X:%.2f Y:%.2f HA:%.2f S:%.4f\n", atlasmv_status->x, atlasmv_status->y, carmen_radians_to_degrees(atlasmv_status->orientation),atlasmv_status->distance_traveled);

		atlasmv_status->header.stamp = ros::Time::now();
		
		/*Publicar status*/
		
		pub_status.publish(*atlasmv_status);
		
		ros::spinOnce();
		LoopRate.sleep();
	}
}


/**
 * @brief Initialize car motion model
 * @param model Motion model to use
 * @param X0 initial x position
 * @param Y0 initial y position
 * @param orientation initial orientation
 * @param wheelbase wheelbase of the car
 * @return error code
 */
int initialize_motion_model(t_motion_model*model,double X0,double Y0,double orientation,double wheelbase)
{
	model->X=X0;
	model->Y=Y0;
	model->HA=orientation;
	model->L=wheelbase;
	model->S=0;
	model->R=NAN;
	return 0;
}

/**
 * @brief Updates velocity and steering variables
 * @param model Motion model to use
 * @param velocity current velocity
 * @param steering_angle current steering angle
 * @return error code
 */
int update_motion_model(t_motion_model*model,double velocity,double steering_angle)
{
	model->V=velocity;
	model->SA=steering_angle;
	return 0;
}

/**
 * @brief Increments motion model based on the current available information
 * @param model Motion model to use
 * @param dt time lapse since last iteration
 * @return error code
 */
int increment_motion_model(t_motion_model*model,double dt)
{
	double R=model->L/tan(model->SA);
	double S=model->V*dt;
	double dHA=S/R;
	
	model->R=R;
	model->S+=S;
	model->HA+=dHA;
	
	if(model->HA > 2*M_PI)
		model->HA-=2*M_PI; 
	if(model->HA < -2*M_PI)
		model->HA+=2*M_PI;
	
	double dXl=R*(sin(dHA));
	double dYl=R*(1-cos(dHA));
	
	if(dHA==0)//This means that the car is moving forward and that R is inf
	{
		dXl=S;
		dYl=0;
	}
	
	model->X+=cos(model->HA)*dXl-sin(model->HA)*dYl;
	model->Y+=sin(model->HA)*dXl+cos(model->HA)*dYl;
	return 0;
}

/** 
* @brief writes used flags for help.
* 
* @param help true if we want to show the help, false if not
*/
void atlasmv_help(bool help)
{
	if(help)
	{
		std::cout<<"\33[36mATLASMV\33[033m\n";
		std::cout<<"command line options:\n";
		std::cout<<"* -debug:\n";
		std::cout<<"    enter in debugging mode, display relevant information while running\n";
		std::cout<<"    show msg frame sent/received from:\n";
		std::cout<<"    DES servoamplifier, example:\n";
		std::cout<<"       request DES version:\n";
		std::cout<<"       sent: | OpCode | len-1 | data[0]|   crc  |\n";
		std::cout<<"             |  0x1A  |  0x00 | 0X0000 | 0x730c |\n";
		std::cout<<"\n";
		std::cout<<"       received: | OpCode | len-1 | data[0]| data[1] | data[2] | data[3] |   crc  |\n";
		std::cout<<"                 |  0x03  |  0x03 | 0x1050 | 0x4101  |  0x00A1 | 0x0001  | 0x7902 |\n";
		std::cout<<"\n";
		std::cout<<"    DIOC discrete I/O:\n";
		std::cout<<"      prints received value and sent value\n";
		std::cout<<"    Hitec 5890gs servos:\n";
		std::cout<<"      print out the value when set new position and read actual position\n";
		std::cout<<"* -h or --help: help menu\n";

		raise(SIGINT);
	}
}
