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
/** @file class_dioc.cpp
* @brief Main source code form the class_doic class.
*/

#include <atlasmv_base/class_dioc.h>

class_dioc::class_dioc(const char*pdevice)
{
	//Get the device
	device=(char*)malloc((strlen(pdevice)+10)*sizeof(char));
	strcpy(device,pdevice);
	
	active=false;//comm off-line
	
	//Comm config params
	struct termios params;
	int ret;
	
	memset( &params,0, sizeof(params)); //setting all structure values to zero
	
	//Opens port comm with the servos
	printf("Opening comm %s ... ",device);fflush(stdout);
	port = open( device, O_RDWR | O_NONBLOCK ); //open com device. read write and non blocking modes	
	if(port == -1) //if could not open
	{perror("Failed to open port");return;}
	printf("Done\n");
	
	active=true;//Comm is now active
	
	printf("Set new parameters ... ");fflush(stdout);
	params.c_cflag = B9600 | CS8 | CLOCAL | CREAD | IGNPAR;
	params.c_iflag = IGNPAR;
	params.c_oflag = 0;
	
	ret=tcsetattr(port, TCSANOW, &params ); // set serial communication parameters
	if( ret < 0 )
	{perror("Set serial communication parameters failed");return;}
	printf("Done\n");
	
	return;
}

class_dioc::~class_dioc()
{
	if(active)
		close(port);
}

void class_dioc::CleanBuffer(void)
{
	int ret=1;
	char data;
	
	if(!active)
		return;
	
	while(ret!=0)
	{
		ret=read(port,&data,1);
		if(ret<0)
			return;
	}
}

int class_dioc::SetStatus(id_enum ID, int STATUS)
{
	char msg;
	
	if(STATUS)
		msg = 0x80 | ID;
	else
		msg = ID;
	
	ret = write(port,&msg,sizeof(msg));
	if(ret<0)
	{
		strcpy(err,"Cannot write to port");
		return -3;
	}
	
	return 0;
}

int class_dioc::SetStatus(id_enum ID,status_enum STATUS)
{
	char msg;
	
	if(STATUS == ON || STATUS == BLINK3)
		msg = 0x80 | ID;
	else
		msg = ID;
	
	ret = write(port,&msg,sizeof(msg));
	if(ret<0)
	{
		strcpy(err,"Cannot write to port");
		return -3;
	}
	
	return 0;
}

int class_dioc::CommStatus(void)
{
	unsigned char msg = 0x00;
	unsigned char data;
	int nbytes_to_read=0;
	
	//Clean input buffer
	CleanBuffer();
	
	ret = write(port,&msg,sizeof(msg));
	if(ret<0)
	{
		strcpy(err,"Cannot write to port");
		return -3;
	}
	
	double ts=ros::Time::now().toSec();
	double tl=0;
	while(nbytes_to_read!=1)
	{
		ret=ioctl(port,FIONREAD,&nbytes_to_read); 
		usleep(1000);
		tl=ros::Time::now().toSec()-ts;
		
		if(tl>1)
		{
			strcpy(err,"DIOC is not responding (read timeout)");
			return -1;
		}
	}
	
	ret=read(port,&data,1);
	if(ret<0)
	{
		strcpy(err,"Cannot read responce");
		return -3;
	}
	
	if(data == 0x80)
		return 1;
	else
		return 0;
}

int class_dioc::GetStatus(id_enum ID)
{
	unsigned char msg;
	int nbytes_to_read=0;
	unsigned char data;
	unsigned char status;
	
	switch(ID)
	{
		case CROSS_A:			
		case CROSS_B:
			msg=ID;
			break;
		default:
			strcpy(err,"This IO does not respond its current state");
			return -1;
	}
	
	//Clean input buffer
	CleanBuffer();
	
	ret = write(port,&msg,sizeof(msg));
	if(ret<0)
	{
		strcpy(err,"Cannot write to port");
		return -3;
	}
	
	double ts=ros::Time::now().toSec();
	double tl=0;
	while(nbytes_to_read!=1)
	{
		ret=ioctl(port,FIONREAD,&nbytes_to_read); 
		usleep(1000);
		tl=ros::Time::now().toSec()-ts;
		
		if(tl>1)
		{
			strcpy(err,"DIOC is not responding (read timeout)");
			return -1;
		}
	}
	
	ret=read(port,&data,1);
	if(ret<0)
	{
		strcpy(err,"Cannot read responce");
		return -3;
	}
	
	status = (data & 0x80) >> 7;
	
	return status;
}

void class_dioc::perr(int ret)
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
