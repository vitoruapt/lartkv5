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
 * @file
 * @brief Class hitec_5980SG implementation
 */

#include <hitec5980sg/hitec5980sg.h>

hitec_5980SG::hitec_5980SG(const char*pdevice)
{
  //Get the device
  device=(char*)malloc((strlen(pdevice)+10)*sizeof(char));
  strcpy(device,pdevice);

  //Set the standard center, minimum and maximum position
  minimum=600;
  center=1500;
  maximum=2400;
  HSR_5980SG_MAX_ANGULAR_SPEED=340.5; /*deg/second @6V feed (calculated by W.Lage[2011] for HSR-5980SG) [3.525% inferior to manufacturer speed]*/
  HSR_5498SG_MAX_ANGULAR_SPEED=263.12; /* supposed real speed | hitec value is 0.22s/60deg=272.72deg/s @6V on wich I apply the 3.525% factor*/
  active=false;//comm off-line

  //Comm config params
  struct termios params;
  int ret;

  memset( &params,0, sizeof(params)); //setting all structure values to zero

  //Opens port comm with the servos
  //          printf("Opening COMM %s ... \n",device);fflush(stdout);
  port = open( device, O_RDWR | O_NONBLOCK ); //open com device. read write and non blocking modes  
  if(port == -1) //if could not open
  {
    perror("Failed to open port");
    return;
  }
  //          printf("Done\n");

  //          printf("Setting new parameters ... ");fflush(stdout);
  params.c_cflag = B19200 | CS8 | CLOCAL | CREAD | CSTOPB | IGNPAR;
  params.c_iflag = IGNPAR;
  params.c_oflag = 0;

  ret=tcsetattr(port, TCSANOW, &params ); // set serial communication parameters
  if( ret < 0 )
  {
    perror("Set serial communication parameters failed");
    return;
  }
  
  //          printf("Done\n");
  fflush(stdout);

  // send dummy instruction to activate comms
  char msg[7];
  //Create message
  msg[0]=0x80;
  msg[1]=0x00;
  msg[2]=0x00;
  msg[3]=0x00;
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
  {
    perror("Serial communication setup failed...");
    return;
  }

  active=true;//Comm is now active

  return;
}

hitec_5980SG::~hitec_5980SG()
{
  try
  {
    if(active)
      close(port);
  }
  catch(char * str)
  {
    perror("Error closing COMM port.");
    perror(str);
    perror("Retrying...");
    try
    {
      close(port);
    }
    catch(char * str)
    {
      perror(str);
      perror("FAILED TO CLOSE COMM.");
    }
  }
}

short unsigned int hitec_5980SG::SetPosition(int id,int position)
{
  int ret;
  char msg[7];
  short unsigned int responce;


  // Check impossibilities
  if(!active)
    return 0xFFFF;

  if(position>maximum)
    return 0xFFFF;
  else if(position<minimum)
    return 0xFFFF;

  //Create message
  msg[0]=0x80;
  msg[1]=id;//id of the servo to move
  msg[2]=position>>8;//position high
  msg[3]=(char)position;//position low
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
  {
    return 0xFFFF;
  }

  responce=ReadResponse();

  return responce;
}

short unsigned int hitec_5980SG::SetSpeedPosition(int id,int speed)
{
  int ret;
  char msg[7];

  if(!active)
    return 0xFFFF;

  //Create message
  msg[0]=0x80;
  msg[1]=0xE9;//command
  msg[2]=id;//id of the servo
  if(speed>255) msg[3]=255; else if (speed<0) msg[3]=0; else msg[3]=speed;//speed
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
      return 0xFFFF;

  usleep(1000); /*IS THIS NEEDED????*/

  ret=ReadResponse();
  if(ret<0 || ret==0xFFFF)
      return 0xFFFF;

  return ret;
}

short unsigned int hitec_5980SG::SetPositionAllServos(short unsigned int position)
{
  int ret;
  char msg[7];
  short unsigned int responce;

  // Check impossibilities
  if(!active)
      return 0xFFFF;

  if(position>maximum)
      return 0xFFFF;

  if(position<minimum)
      return 0xFFFF;

  //Create message
  msg[0]=0x80;
  msg[1]=0xE6; /*COMMAND TO ORDER ANY SERVO TO COMPLY*/
  msg[2]=position>>8;//position high
  msg[3]=(char)position;//position low
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
      return 0xFFFF;

  responce=ReadResponse();

  return responce;
}

short int hitec_5980SG::GetVersionAndID(void)
{
  int ret;
  char msg[7];

  char data[7];
  int nbytes_to_read=0;

  //Create message
  msg[0]=0x80;
  msg[1]=0xE7; /*COMMAND TO OUTPUT ID NUMBER*/
  msg[2]=0;
  msg[3]=0;
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
      return 0xFFFF;

  if(!active)
      return 0xFFFF;

  /// \todo You should not do this!, there is a function called ReadResponse to do this part!!
  while(nbytes_to_read!=7)
  {
      ret=ioctl(port,FIONREAD,&nbytes_to_read);
      usleep(1000);
  }

  ret=read(port,&data,7);
  if(ret!=7)
      return 0xFFFF;

  return (0x00FF & data[6]);
}

short unsigned int hitec_5980SG::ReleaseServos(void)
{
  int ret;
  char msg[7];
  short int responce;

  // Check impossibilities
  if(!active)
      return 0xFFFF;

  //Create message
  msg[0]=0x80;
  msg[1]=0xEF; /*COMMAND TO POWER DOWN SERVO STATOR*/
  msg[2]=0;
  msg[3]=0;
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
      return 0xFFFF;

  char data[7];
  int nbytes_to_read=0;

  /// \todo You should not do this!, there is a function called ReadResponse to do this part!! | Pedro_Cruz_Response: Indeed! But it doesn't work in this case! Function must be altered to suit all possible responces, because some functions place different information in those two last bytes...
  while(nbytes_to_read!=7)
  {
      ret=ioctl(port,FIONREAD,&nbytes_to_read); 
      usleep(1000);
  }

  ret=read(port,&data,7);
  if(ret!=7)
      return 0xFFFF;

  responce=(0x00FF & data[5]);

  /*DON'T MIND THIS PIECE OF CODE | BUS PROBLEMS...*/
  if(responce!=3)
      return 0xFFFF;
  else
      return 3;
}

short unsigned int hitec_5980SG::SetServoID(int id)
{
  int ret;
  char msg[7];


  char data[7];
  int nbytes_to_read=0;

  //Create message
  msg[0]=0x80;
  msg[1]=0xE2; /*COMMAND TO SET VALUE TO A MEMORY POSITION*/
  msg[2]=0x29; /*ID MEMORY POSITION*/
  msg[3]=id;
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
  {
      return 0xFFFF;
  }

  if(!active)
      return 0xFFFF;

  /// \todo You should not do this!, there is a function called ReadResponse to do this part!! | Pedro_Cruz_Response: Indeed! But it doesn't work in this case! Function must be altered to suit all possible responces, because some functions place different information in those two last bytes...
  while(nbytes_to_read!=7)
  {
      ret=ioctl(port,FIONREAD,&nbytes_to_read); 
      usleep(1000);
  }

  ret=read(port,&data,7);
  if(ret!=7)
      return 0xFFFF;

  return (0x00FF & data[6]);
}

short unsigned int hitec_5980SG::SetGoStop(short unsigned int value)
{
  int ret;
  char msg[7];
  short unsigned int responce;
  int sendvalue;

  // Check impossibilities
  if(!active)
      return 0xFFFF;

  if(value>=1)
      sendvalue=1;
  else
      sendvalue=0;

  //Create message
  msg[0]=0x80;
  msg[1]=0xEB; /*COMMANDO FOR STOP/GO*/
  msg[2]=sendvalue;
  msg[3]=0;
  msg[4]=256 - (0x80 + msg[1] + msg[2] + msg[3])%256;
  msg[5]=0x00;//Return 1
  msg[6]=0x00;//Return 2

  //Clean input buffer
  CleanBuffer();

  //Send message
  ret = write(port,&msg,sizeof(msg));
  if(ret<0)
  {
      return 0xFFFF;
  }

  char data[7];
  int nbytes_to_read=0;

  /// \todo You should not do this!, there is a function called ReadResponse to do this part!! | Pedro_Cruz_Response: Indeed! But it doesn't work in this case! Function must be altered to suit all possible responces, because some functions place different information in those two last bytes...
  while(nbytes_to_read!=7)
  {
      ret=ioctl(port,FIONREAD,&nbytes_to_read); 
      usleep(1000);
  }

  ret=read(port,&data,7);
  if(ret!=7)
      return 0xFFFF;

  responce=(0x00FF & data[5]);
  /*DON'T MIND THIS PIECE OF CODE | BUS PROBLEMS...*/
  if(responce!=3)
      return 0xFFFF;
  else
      return 3;
}

bool hitec_5980SG::IsActive(void)
{
  return active;
}

short unsigned int hitec_5980SG::ReadResponse(char*response_1,char*response_2)
{
  int ret=1;
  char data[7];
  int nbytes_to_read=0;
  double diff_time;
  time_t start,end;

  if(!active)
      return 0xFFFF;

  time (&start);
  while(nbytes_to_read!=7)
  {
      ret=ioctl(port,FIONREAD,&nbytes_to_read); 
      usleep(1000);
      time (&end);
      diff_time=difftime ( end, start);
      if(diff_time>1.)
          return 0xFFFF; /*CONNECTION TIMED OUT | 1_SEC SEEMS TOO MUCH BUT I'LL KEEP IT...*/
  }

  ret=read(port,&data,7);
  if(ret!=7)
      return 0xFFFF;
  
  if(response_1 && response_2)
  {
    *response_1=data[5];
    *response_2=data[6];
  }
  
  return (0xFF00 & (data[5]<<8)) | (0x00FF & data[6]);
}

void hitec_5980SG::CleanBuffer(void)
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

short unsigned int hitec_5980SG::ConvertAngularSpeedToServoSpeed(double angular_speed, unsigned int Servo_type)
{
  //choose servo motor type
  double max_speed;
  short unsigned int return_value;
  if(Servo_type==HSR_5980SG)
  {
    max_speed=HSR_5980SG_MAX_ANGULAR_SPEED;
  }
  else if(Servo_type==HSR_5498SG)
  {
    max_speed=HSR_5498SG_MAX_ANGULAR_SPEED;
  }
  else
    return 1;
  
  //convert servo digital speed
  if(fabs(angular_speed)>=max_speed){
    return 255;}
  else{
    return_value = round((255./max_speed) * fabs(angular_speed));}
  
  if(return_value<1){
    //means the servo will not move [STOP ZONE]
    return 0xFFFF;}
  else
      return return_value;
}
