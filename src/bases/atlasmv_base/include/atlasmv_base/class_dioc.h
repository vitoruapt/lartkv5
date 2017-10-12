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
/** @file class_dioc.h
* @brief Includes, global vars, function prototypes, etc.
*/

#ifndef _DIOC_H_
#define _DIOC_H_

//System Includes
//all system includes go here
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

/**
@brief This class implements the DIOC communication protocol.

This class allows to control the discreet input/output PCB.
*/
class class_dioc
{
	public:
		/**
		@brief Constructor
		
		This functions sets the default values for internal variables and opens communication with the servos.
		@param pdevice Name of the device to use in communications
		@return void
		*/
		class_dioc(const char*pdevice);
		
		/**
		@brief De-constructor
		
		This function closes the communication with the servos.
		@return void
		*/
		~class_dioc();
		
		typedef enum {ON,OFF,BLINK1,BLINK3} status_enum;
		typedef enum {STATUS=0,FRONT,RIGHT,LEFT,REVERSE,BRAKE,BLUE,RED,GREEN,CROSS_A,CROSS_B} id_enum;
		
		/**
		@brief Set the status of a input/output
		
		@param ID id of the IO
		@param ENUM required status
		@return error code
		*/
		int SetStatus(id_enum ID,status_enum ENUM);
		
		/**
		@brief Set the status of a input/output
		
		@param ID id of the IO
		@param status required status
		@return error code
		*/
		int SetStatus(id_enum ID, int status);
		
		/**
		@brief Get the status of a input/output
		@param ID id of the IO
		@return error code
		*/
		int GetStatus(id_enum ID);
		
		/**
		@brief Get the comm status
		@return error code
		*/
		int CommStatus(void);
		
		/**
		@brief Print error function
		
		This function prints the error present in the err variable (this is a private variable of
		the gamepad class). The ret number specifies what kind of error is this, -1 is a error that
		raises a SIGINT signal, -2 is an warning that allows the program to continue and -3 is a 
		error that prints perror and raises SIGINT.
		@param ret error code
		@return void
		*/
		void perr(int ret);
		
	private:
		
		/**
		@brief Cleans the buffer
		
		This function clean the communication input buffer.
		@return void
		*/
		void CleanBuffer(void);
		
		///This variable indicates that the communication is active.
		bool active;
		
		///Communication port to use
		int port;
		
		///Auxiliary return code
		int ret;
		
		///Communication device to use
		char*device;
		
		char err[1024];
};
#endif
