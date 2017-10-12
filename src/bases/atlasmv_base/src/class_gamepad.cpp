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
 * \brief Gamepad communication class source code generic module
 */

#include <atlasmv_base/class_gamepad.h>

class_gamepad::class_gamepad()
{
	ret=0;
	fd=0;
	n_buttons=0;
	n_axes=0;
}

class_gamepad::~class_gamepad()
{
	close(fd);
}

int class_gamepad::StartComm(const char*device)
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
	
	buttons=(t_button*)malloc(n_buttons*sizeof(t_button));
	
	axes=(t_button*)malloc(n_axes*sizeof(t_button));
	
	for(int i=0;i<n_buttons;i++)
		buttons[i].callback=NULL;
	
	for(int i=0;i<n_axes;i++)
		axes[i].callback=NULL;
	
	return 0;
}

int class_gamepad::GetButtonMapping(void)
{
	ret=ioctl(fd,JSIOCGAXMAP,&m_axes);
	if(ret<0)
	{
		strcpy(err,"Unable to read axes mapping");
		return -3;
	}
	
	ret=ioctl(fd,JSIOCGBTNMAP,&m_buttons);
	if(ret<0)
	{
		strcpy(err,"Unable to read buttons mapping");
		return -3;
	}
	
	return 0;	
}

int class_gamepad::SetButtonMapping(void)
{
	ret=ioctl(fd,JSIOCSAXMAP,&m_axes);
	if(ret<0)
	{
		strcpy(err,"Unable to set axes mapping");
		return -3;
	}
	
	ret=ioctl(fd,JSIOCSBTNMAP,&m_buttons);
	if(ret<0)
	{
		strcpy(err,"Unable to set buttons mapping");
		return -3;
	}
	
	return 0;	
}

int class_gamepad::RegisterCallback(e_type type,int id,void (*callback)(int value,void*data),void*data)
{
	if(type==AXIS)
	{
		if(id>n_axes)
		{
			sprintf(err,"This device does not have this axis: Axis %d",id);
			return -2;
		}
		
		axes[id].callback=callback;
		axes[id].userdata=data;
		
	}else if(type==BUTTON)
	{
		if(id>n_buttons)
		{
			sprintf(err,"This device does not have this Button: Button %d",id);
			return -2;
		}
		
		buttons[id].callback=callback;
		buttons[id].userdata=data;
	}
	
	
	return 0;
}

int class_gamepad::Dispatch(bool debug)
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
			if(buttons[buffer[i].number].callback==NULL)
				continue;
			
			buttons[buffer[i].number].value = buffer[i].value;
			buttons[buffer[i].number].callback(buttons[buffer[i].number].value,buttons[buffer[i].number].userdata);
		}else if(buffer[i].type == JS_EVENT_AXIS)
		{
			if(debug)printf("Axis %d Value %d\n",buffer[i].number,buffer[i].value);
			if(axes[buffer[i].number].callback==NULL)
				continue;
			
			axes[buffer[i].number].value = buffer[i].value;
			axes[buffer[i].number].callback(axes[buffer[i].number].value,axes[buffer[i].number].userdata);
		}
	}
	
	return 0;
}

void class_gamepad::plerr(int ret)
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
