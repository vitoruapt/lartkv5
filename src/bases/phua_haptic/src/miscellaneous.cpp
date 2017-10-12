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
 * \brief Miscellaneous functions implementation
 */

#include <phua_haptic/miscellaneous.h>

/*~~~~~~~~~~~~~~~~~ 
||      help      ||
~~~~~~~~~~~~~~~~~~*/
void print_help(void)
{
  const char help[]={"***************************************************************************+\n\
		    ------------------------------			   |\n\
		    | PHUA HAPTIC INTERFACE HELP |			   |\n\
		    ------------------------------			   |\n\
									   |\n\
  This program uses RS232 comunication over an USB adapter to send	   |\n\
  commands to the humanoid robot.					   |\n\
  If you're having trouble connecting, use the path to your USB port	   |\n\
  as an argument when you call the application.				   |\n\
									   |\n\
  example: >> ./phua_haptic_interface /dev/ttyUSB*			   |\n\
	      [where * is the number of your port]			   |\n\
									   |\n\
---------------------------------------------------------------------------+\n\
									   |\n\
  The PHANToM OMNI joystick functions require a FireWire port active	   |\n\
  and superuser permissions to access it.				   |\n\
  If you are having trouble with this, open a terminal and run:		   |\n\
									   |\n\
  >> sudo modprobe raw1394						   |\n\
									   |\n\
  This will probe the FireWire port the joystick connection. If it is      |\n\
  sucessful, module raw1394 should appear in your ´/dev/´ system folder.   |\n\
									   |\n\
  For the program to have permission to communicate with the device, call  |\n\
  this application with superuser rights using ´sudo´ or run within	   |\n\
  a terminal:								   |\n\
									   |\n\
  >> sudo chmod 777 /dev/raw1394					   |\n\
									   |\n\
  This should allow normal execution of the program.			   |\n\
									   |\n\
---------------------------------------------------------------------------+\n\
									   |\n\
  To run aplication in debug mode, use argument ´--debug´.		   |\n\
  In this mode, application will execute even if connections are	   |\n\
  not established. Keep in mind that the application may not		   |\n\
  properly control the robot or retrieve data from the PHANToM OMNI	   |\n\
  joystick in this mode, and a restart is required in order to		   |\n\
  use the program's full funcionalities.				   |\n\
									   |\n\
  example: >> ./phua_haptic_interface --debug				   |\n\
									   |\n\
---------------------------------------------------------------------------+\n\
									   |\n\
  Pedro Cruz 2012							   |\n\
  Happy Haptics! :-)							   |\n\
***************************************************************************+"
  };
  printf("%s\n",help);
}

void print_greeting(shared_vars_t*RobotVars)
{
  char servo_comm_on[15]=" ";
  if(RobotVars->servo->IsActive())
    strcat(servo_comm_on,"ONLINE");
  else
    strcat(servo_comm_on,"OFFLINE");
  
  char phant_on[15]=" ";
  if(RobotVars->phantom_data.phantom_on)
    strcat(phant_on,"ONLINE");
  else
    strcat(phant_on,"OFFLINE");
  
 printf("***************************************************************************\n\
		    -------------------------------------		   \n\
		    | PHUA HAPTIC INTERFACE APPLICATION |		   \n\
		    -------------------------------------		   \n\
									   \n\
  Pedro Cruz 2012							   \n\
***************************************************************************\n\
									   \n\
     REQUIRED COMUNICATION STATUS					   \n\
  ----------------------------------					   \n\
  > Servomotor COMM:	|%s |						   \n\
  > PHANToM OMNI COMM:	|%s |						   \n\
  ----------------------------------					   \n",
										servo_comm_on,phant_on);
}

void CalculateAverageCartesianSpeed(double diffX, double diffY, double diffZ, long double time_interval, double *return_vector)
{ 
  static const uint array_size=20;
  
  static std::vector<double> accumulationXdiff;
  static std::vector<double> accumulationYdiff;
  static std::vector<double> accumulationZdiff;
  static std::vector<long double> accumulation_time;
  
  accumulationXdiff.push_back(diffX);
  accumulationYdiff.push_back(diffY);
  accumulationZdiff.push_back(diffZ);
  accumulation_time.push_back(time_interval);
  
  if(accumulationXdiff.size()>array_size)
	  accumulationXdiff.erase(accumulationXdiff.begin());
  
  if(accumulationYdiff.size()>array_size)
	  accumulationYdiff.erase(accumulationYdiff.begin());
  
  if(accumulationZdiff.size()>array_size)
	  accumulationZdiff.erase(accumulationZdiff.begin());
  
  if(accumulation_time.size()>array_size)
	  accumulation_time.erase(accumulation_time.begin());
  
  double x_mean=pc_mean<double>(accumulationXdiff);
  double y_mean=pc_mean<double>(accumulationYdiff);
  double z_mean=pc_mean<double>(accumulationZdiff);
  long double time_mean=pc_mean<long double>(accumulation_time);
  
  return_vector[0]=round(x_mean/time_mean);
  return_vector[1]=round(y_mean/time_mean);
  return_vector[2]=round(z_mean/time_mean);
}

Eigen::Matrix3d rotx(double angle_in_degrees)
{
  static Eigen::Matrix3d rotation_matrix;
  static double angle_in_radians;
  
  angle_in_radians = DegToRad(angle_in_degrees);
  
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      if((i==0 && j==0))
	rotation_matrix(i,j) = 1;
      else if((i==1 && j==1) || (i==2 && j==2))
	rotation_matrix(i,j) = cos(angle_in_radians);
      else if((i==1 && j==2))
	rotation_matrix(i,j) = -1 * sin(angle_in_radians);
      else if((i==2 && j==1))
	rotation_matrix(i,j) = sin(angle_in_radians);
      else
	rotation_matrix(i,j) = 0;
    }
  }
  return rotation_matrix;
}

Eigen::Matrix3d roty(double angle_in_degrees)
{
  static Eigen::Matrix3d rotation_matrix;
  static double angle_in_radians;
  
  angle_in_radians = DegToRad(angle_in_degrees);
  
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      if(i==1 && j==1)
	rotation_matrix(i,j) = 1;
      else if((i==0 && j==0) || (i==2 && j==2))
	rotation_matrix(i,j) = cos(angle_in_radians);
      else if((i==0 && j==2))
	rotation_matrix(i,j) = sin(angle_in_radians);
      else if((i==2 && j==0))
	rotation_matrix(i,j) = -1. * sin(angle_in_radians);
      else
	rotation_matrix(i,j) = 0;
    }
  }
  return rotation_matrix;
}

Eigen::Matrix3d rotz(double angle_in_degrees)
{
  static Eigen::Matrix3d rotation_matrix;
  static double angle_in_radians;
  
  angle_in_radians = DegToRad(angle_in_degrees);
  
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      if(i==2 && j==2)
	rotation_matrix(i,j) = 1;
      else if((i==0 && j==0) || (i==1 && j==1))
	rotation_matrix(i,j) = cos(angle_in_radians);
      else if((i==1 && j==0))
	rotation_matrix(i,j) = sin(angle_in_radians);
      else if((i==0 && j==1))
	rotation_matrix(i,j) = -1. * sin(angle_in_radians);
      else
	rotation_matrix(i,j) = 0;
    }
  }
  return rotation_matrix;
}

double pc_mean(double *v, int len)
{
  double sum = 0;
  int i;
  for (i = 0; i < len; i++)
	  sum += v[i];
  return sum / len;
}

int isNumeric (const char * s)
{
  if (s == NULL || *s == '\0' || isspace(*s))
    return 0;
  char * p;
  int r=strtod (s, &p);
  if(r){}
  return *p == '\0';
}

double get_sign(double x)
{
  return ((x >= 0.) ? 1. : -1.);
}

int GetNbrFromKeyboard(void)
{
  int s;
  int ret=scanf("%d",&s);
  if(ret){}
  return s;
}

char GetStrFromKeyboard(void)
{
  char s;
  int ret=scanf("%s",&s);
  if(ret){}
  return s;
}

double GetDblFromKeyboard(void)
{
  float s;
  int ret=scanf("%f",&s);
  if(ret){}
  return s;
}

void ScreenClear(void)
{
  printf("\033[2J");
  printf("\033[0;0f");
}

void textcolor(int attr, int fg, int bg)
{
  printf("%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
}

void PrintRedLine(void)
{
  int n;
  textcolor(RESET, RED, WHITE);
  for(n=1;n<41;n++)
	  printf("=");
  printf("\n");
}

void ResetTextColors(void)
{
  textcolor(RESET, WHITE, WHITE);
}

void HighLightText(void)
{
  textcolor(BRIGHT, WHITE, BLACK);
}
