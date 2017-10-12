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
* @addtogroup timer
*@{
	*/
#ifndef _TIMER_CPP_
#define _TIMER_CPP_
/** @file
* @brief Main sorce code from the c_timer class.
*/

#include <atlasmv_base/timer.h>

/** 
* @brief function to get time
* 
* @return actual time in double format in seconds with a precision of 0.01sec
*/
double lar_get_time()
{
	struct timeval tv;
	double t;
	
	if(gettimeofday(&tv, NULL)<0)
	{
		printf("\33[31mWARNING:\33[0m problem getting time\n");
	}
	
	t = (tv.tv_sec + (tv.tv_usec/1000000.0));// function gettimeofday returns time in seconds with a precision of 0.01seconds
	
	return t;
}
/** 
* @brief runned when created class
*/
c_timer::c_timer()
{
	// 	printf("class initialized\n");
	for (int j=0;j<10;j++)
	{
		clock_set[j] = 0; //set all ipc flags to zero
		lar_toc[j] = 0;
		lar_tic[j] = 0.0;
	}
	//if(lar_ipc_sleep(0.01))
	//{
		//ipc_connected=0;
		//}else
		//{
			//ipc_connected=1;
			//};
			
			//pthread_mutex_init(&ipc_connected_mutex, NULL);
			pthread_mutex_init(&clock_tic_mutex, NULL);
			pthread_mutex_init(&clock_set_mutex, NULL);
			pthread_mutex_init(&lar_tic_mutex, NULL);
			pthread_mutex_init(&lar_toc_mutex, NULL);
			
};

/** 
* @brief method ruuned when class is destroyed
*/
c_timer::~c_timer()
{
}

/** 
* @brief method with run_ipc_sleep that allows that to set a refresh rate of a specified code.
* 
* @param i - number of the ipc_sleep, it can exist 10 at maximum
*/
void c_timer::set_ipc_sleep(int i)
{
	pthread_mutex_lock(&clock_set_mutex);
	if(!clock_set[i])
	{
		clock_tic[i] = lar_get_time();
		clock_set[i] = 1;
	}
	pthread_mutex_unlock(&clock_set_mutex);
};

/** 
* @brief method to use inside thread to avoid collisions between threads
* 
* @param i - the same number of used when called tic
* @param desired_freq - frequency in Hz
*/
void c_timer::run_sleep(int i, double desired_freq)
{
	double ts = (1/desired_freq);
	double tsf;
	unsigned int t;
	
	if(abs(i)<11)
	{
		//pthread_mutex_lock(&lar_toc_mutex);
		//if(!lar_toc[i])
		//{
			//pthread_mutex_unlock(&lar_toc_mutex);
			//toc(i);
			//}
			pthread_mutex_lock(&lar_tic_mutex);
			tsf = ts - (ros::Time::now().toSec()-lar_tic[i]);
			pthread_mutex_unlock(&lar_tic_mutex);
			t=(tsf<0?100:tsf*1E6);
			//printf("sleep:%d\n", t);
			usleep(t);
}

};

/** 
* @brief to use this method is required to run set_ipc_sleep method previously
* 
* @param desired_freq - frequency to execute a specified code
* @param i - number of timer
* @param verbose - 
* 
* @return 
*/
char c_timer::run_ipc_sleep(double desired_freq, int i, char verbose)
{
// 	double ts = (1.0/desired_freq);
// 	double tsf = ts;
// 	
// 	pthread_mutex_lock(&clock_set_mutex);
// 	if(clock_set[i])
// 	{
// 		tsf = ts - (ros::time:now().to_sec()-clock_tic[i]);
// 		
// 		if(tsf <= MINIMUM_IPC_SLEEP)
// 		{
// 			if(verbose)
// 			{
// 				printf("\33[31mdesired frequency %3.1f NOK.\33[0m\n", desired_freq);
// 				printf("\33[31mactual frequency %3.2f.\33[0m\n", 1.0/(ros::time:now().to_sec()-clock_tic[i]));
// 			};
// 			tsf = MINIMUM_IPC_SLEEP;
// 		}else
// 		{
// 			if(verbose)
// 			{
// 				printf("\33[31mdesired frequency %3.1fHz OK.\33[0m\r", desired_freq); fflush(stdout);
// 				printf("\33[31mactual frequency %3.2f.\33[0m\n", 1.0/(ros::time:now().to_sec()-clock_tic[i]));
// 			}
// 		};
// 	}
// 	
// 	
// 	//carmen_ipc_sleep(tsf);
// 	char rt = lar_ipc_sleep(tsf);
// 	clock_set[i]=0;
// 	pthread_mutex_unlock(&clock_set_mutex);

	return 'A';
}

/** 
* @brief method when used with toc enables you to know how many time have passed since
* 
* @param i - integer values from 0 to 10;
*/
void c_timer::tic(int i)
{
	pthread_mutex_lock(&lar_tic_mutex);
	if(abs(i)<11)
	{
		lar_tic[i] = ros::Time::now().toSec();
	}
	pthread_mutex_unlock(&lar_tic_mutex);
};

/** 
* @brief method when used with t1c enables you to know how many time have passed since
* 
* @param i - integer values from 0 to 10;
*/
void c_timer::toc(int i)
{
	pthread_mutex_lock(&lar_toc_mutex);
	pthread_mutex_lock(&lar_tic_mutex);
	if(abs(i)<11)
		lar_toc[i] = (ros::Time::now().toSec() - lar_tic[i]);
	
	pthread_mutex_unlock(&lar_tic_mutex);
	pthread_mutex_unlock(&lar_toc_mutex);
};

//added by dgameiro to allow get time stamp
double c_timer::get_toc(int i)
{
	if(abs(i)<11)
	{
		pthread_mutex_lock(&lar_toc_mutex);
		if(!lar_toc[i])
		{
			pthread_mutex_unlock(&lar_toc_mutex);
			toc(i);
			pthread_mutex_lock(&lar_toc_mutex);
		}
		double result = lar_toc[i];
		pthread_mutex_unlock(&lar_toc_mutex);
		return result;
	}else{
		// 		printf("error. can only exist 10");
	}
	return 0;
}

#endif
/**
*@}
*/
