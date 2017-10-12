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
* @defgroup timer Timer
* @brief This module implements tictoc matlab like timers.
* @ingroup utils
*@{
	*/
#ifndef _TIMER_H_
#define _TIMER_H_

/** @file
* @brief main header file for this module. Includes, global vars, funtion prototypes, etc.
*/

#define MINIMUM_IPC_SLEEP 0.005

//####################################################################
// Includes:
//####################################################################

//System Includes
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <ros/ros.h>
//Include lar wrappers

//Include lar individual modules


//Include local modules



// double lar_get_time();

class c_timer{
	public:
		c_timer();
		~c_timer();
		
		void tic(int);
		void toc(int);
		void run_sleep(int, double);
		void set_ipc_sleep(int);
		char run_ipc_sleep(double, int, char);
		double get_toc(int);//added by dgameiro on 25-02-2010
		void verb(int start=0, int end=10){printf("class timer tocs...\n");for (int i=start;i<end;i++) {printf("toc[%d]=%3.4f ms\n",i,lar_toc[i]*1000.0);}};
		
	private:
		//pthread_mutex_t ipc_connected_mutex; 
		char ipc_connected;
		pthread_mutex_t clock_tic_mutex;
		double clock_tic[10];
		pthread_mutex_t clock_set_mutex;
		char clock_set[10];
		pthread_mutex_t lar_tic_mutex;
		double lar_tic[10];
		pthread_mutex_t lar_toc_mutex;
		double lar_toc[10];
};
#endif

/**
*@}
*/
