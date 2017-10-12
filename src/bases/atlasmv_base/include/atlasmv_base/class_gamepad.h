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
 * \brief Gamepad communication class declaration generic code
 */

#ifndef _CLASS_GAMEPAD_H_
#define _CLASS_GAMEPAD_H_

#include <linux/joystick.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>

class class_gamepad
{
	public:
		
		typedef enum {AXIS,BUTTON} e_type;
		
		/**
		* \brief Class constructor.
		*/
		class_gamepad();
		
		/**
		* \brief Class destructor.
		*/
		~class_gamepad();
		
		/**
		* @brief Initialize comm with the gamepad
		* @param device Name of the device to use
		* @return error code that can be analyzed with the function plerr
		*/
		int StartComm(const char*device);
		
		/** 
		* @brief Register a callback for a specific button or axis
		* @param type AXIS or BUTTON (these enums are present in the gamepad class)
		* @param id number of the button or axis * @param callback function to be called prototype is void (*callback)(int value,void*data), where the value is the button or axis value and data is the user data
		* @param data user data to pass to the callback
		* @return error code that can be analyzed with the function plerr
		*/
		int RegisterCallback(e_type type,int id,void (*callback)(int value,void*data),void*data);
		int UnRegisterCallback(e_type type,int id);
		
		/**
		* @brief This function checks the gamepad status and calls the respective callback
		* @param debug if set to true this function will print all events
		* @return error code that can be analyzed with the function plerr
		*/
		int Dispatch(bool debug=false);
		int CloseComm(void);
		
		/**
		* @brief Print local error function
		This function prints the error present in the err variable (this is a private variable of	the gamepad class). The ret number specifies what kind of error is this, -1 is a error that raises a SIGINT signal, -2 is an warning that allows the program to continue and -3 is a error that prints perror and raises SIGINT.
		* @param ret error code
		* @return void
		*/
		void plerr(int ret);
	private:
		
		typedef struct
		{
			int value;
			void *userdata;
			void (*callback)(int value,void*data);
		}t_button;
		
		/**
		* @brief Get the button mapping from the device
		* @return error code that can be analyzed with the function plerr
		*/
		int GetButtonMapping(void);
		
		/**
		* @brief Set the button mapping to the device
		* @return error code that can be analyzed with the function plerr
		*/
		int SetButtonMapping(void);
		
		int ret;
		int fd;
		int n_buttons;
		int n_axes;
		t_button*buttons;
		t_button*axes;
		
		__u8 m_axes[ABS_CNT];
		__u16 m_buttons[KEY_MAX - BTN_MISC + 1];
		
		char name[1024];
		char err[1024];
		char text[1024];
};

#endif
