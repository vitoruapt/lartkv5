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
#ifndef _GAMEPAD_H_
#define _GAMEPAD_H_

/**
\file
\brief Generic gamepad operation class
*/

#include <linux/joystick.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <vector>

#include <sigc++/functors/slot.h>
#include <sigc++/functors/ptr_fun.h>
#include <sigc++/functors/mem_fun.h>
#include <boost/shared_ptr.hpp>

/**
 * @file 
 * @brief Gamepad class definition
\page "Gamepad class use example (using libsigc++)"
\anchor expage
\section example How to use the Gamepad in your project?

In the beginning of your code create a class instance and start the communication on the correct
device.
@code
Gamepad gamepad;//Create the instance
gamepad.StartComm("/dev/input/js0");//Connect to the device
@endcode

After declaration, register callbacks on buttons and axes.
@code

gamepad.buttons(7)->callback = sigc::mem_fun<int>(*this,&CallbackClass::gamepadIgnitionONCallback);//Registry a callback on an axes, this callback is a member function, if the callback is not a member function use sigc::ptr_fun instead

@endcode

Example of a callback function:
@code
void CallbackClass::GamepadThrottle(int value)
{
	double throttle;
	throttle=(((double)(value)/32768. + 1)/2);
	if(throttle<0.2)
		throttle=throttle*3;
	else
		throttle=throttle/2+0.5;
	
	if(throttle>1)
		throttle=1;
	command.throttle=throttle;
}
@endcode

In the main loop of your code you must call the function \c Dispatch() in order to get new data from the device, this function automatically calls the respective callbacks when events occur.
@code
gamepad.Dispatch(false);//Read device and call handlers, use Dispatch(true) to run in verbose mode (default is false)
@endcode
*/


///Button global structure
class Button
{
	public:
		Button()
		{
			value=0;
		}
		///value of the button
		int value;
		
		///callback to be used when the button changes state
		sigc::slot<void, int> callback;
};

typedef boost::shared_ptr<Button> ButtonPtr;

/**
 * \brief Gamepad class
 * 
 * 
 */
class Gamepad
{
	public:
		///Type of input available on the game pad
		typedef enum {AXIS,BUTTON} e_type;
		
		/**
		@brief Class constructor.
		*/
		Gamepad():
		ret(0),
		fd(0),
		n_buttons(0),
		n_axes(0)
		{
		}
		
		/**
		@brief Class destructor.
		*/
		~Gamepad()
		{
			close(fd);
		}
			
		
		/**
		@brief Initialize comm with the gamepad
		
		@param device Name of the device to use
		@return error code that can be analyzed with the function plerr
		*/
		int startComm(const char*device)
		{
			fd = open(device, O_RDONLY | O_NONBLOCK);
			if(fd<0)
			{
				sprintf(err,"Unable to open device: %s",device);
				return -3;
			}
			
			ret=ioctl(fd, JSIOCGAXES, &n_axes);
			if(ret<0)
			{
				strcpy(err,"Unable to read number of axes");
				return -3;
			}
			
			ret=ioctl(fd, JSIOCGBUTTONS, &n_buttons);
			if(ret<0)
			{
				strcpy(err,"Unable to read number of buttons");
				return -3;
			}
			
			ret=ioctl(fd, JSIOCGNAME(sizeof(name)), name);
			if(ret<0)
				strcpy(name,"Unknown");
			
			printf("\33[1m\33[36mGamepad\33[0m %s\n",name);
			
			for(int i=0;i<n_buttons;i++)
			{
				ButtonPtr button(new Button);
				buttons_.push_back(button);
			}
			
			for(int i=0;i<n_axes;i++)
			{
				ButtonPtr button(new Button);
				axes_.push_back(button);
			}
			
			return 0;
		}

		/**
		@brief This function checks the gamepad status and calls the respective callback
		@param debug if set to true this function will print all events
		@return error code that can be analyzed with the function plerr
		*/
		int dispatch(bool debug=false)
		{
			struct js_event buffer[64];
			
			ret = read (fd, buffer, sizeof(struct js_event)*64);
			if(ret<0)
				return 0;
			
			for(int i=0; i<ret/(signed int)sizeof(struct js_event);i++)
			{
				if(buffer[i].type & JS_EVENT_BUTTON & ~JS_EVENT_INIT)
				{
					if(debug)printf("Button %d Value %d\n",buffer[i].number,buffer[i].value);
					if(buttons_[buffer[i].number]->callback.empty())
						continue;
					
					buttons_[buffer[i].number]->value = buffer[i].value;
					buttons_[buffer[i].number]->callback(buttons_[buffer[i].number]->value);
				}else if(buffer[i].type == JS_EVENT_AXIS)
				{
					if(debug)printf("Axis %d Value %d\n",buffer[i].number,buffer[i].value);
					if(axes_[buffer[i].number]->callback.empty())
						continue;
					
					axes_[buffer[i].number]->value = buffer[i].value;
					axes_[buffer[i].number]->callback(axes_[buffer[i].number]->value);
				}
			}
			
			return 0;
		}
		
		/**
		@brief Print local error function
		
		This function prints the error present in the err variable (this is a private variable of
		the \c class_gamepad). The ret number specifies what kind of error is this, -1 is a error that
		raises a SIGINT signal, -2 is an warning that allows the program to continue and -3 is a 
		error that prints perror and raises SIGINT.
		
		@param ret error code
		@return void
		*/
		void plerr(int ret)
		{
			if(ret==-1)
			{
				printf("Error!!   ");
				printf("%s (raising SIGINT)\n",err);
				raise(SIGINT);
			}else if(ret==-2)
			{
				printf("Warning!! ");
				printf("%s\n",err);
			}else if(ret==-3)
			{
				printf("Error!!   ");
				printf("%s, ",err);fflush(stdout);
				perror(NULL);
				raise(SIGINT);
			}
		}
		
		
		ButtonPtr buttons(uint i)
		{
			if(i>0 || i<buttons_.size())
				return buttons_[i];
			else
			{
				ButtonPtr empty;
				return empty;
			}
		}
		
		ButtonPtr axes(uint i)
		{
			if(i>0 || i<axes_.size())
				return axes_[i];
			else
			{
				ButtonPtr empty;
				return empty;
			}
		}
		
	private:
		
		///Vector of buttons
		std::vector<ButtonPtr> buttons_;
		///Vector of axes, buttons and axes share the same data structure \c Button
		std::vector<ButtonPtr> axes_;
		
		///Auxiliary error variable 
		int ret;
		
		///File descriptor of the device
		int fd;
		
		///Number of buttons in the game pad
		int n_buttons;
		
		///Number of axes in the game pad
		int n_axes;
		
		///Mapping variable
		__u8 m_axes[ABS_CNT];
		///Mapping variable
		__u16 m_buttons[KEY_MAX - BTN_MISC + 1];
		
		///Name of the game pad
		char name[1024];
		
		///Error message
		char err[1024];
		
		///Auxiliary string variable
		char text[1024];
};
#endif
