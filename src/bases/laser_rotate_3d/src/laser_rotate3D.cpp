/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 LAR-DEM-University of Aveiro.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of LAR-DEM-University of Aveiro nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
*/

/**
*@file laser_rotate3D.cpp
*@brief PICCOMM Node used to communicate with the PIC MCU installed to rotate the laser for 3D
*@author Ricardo Pascoal  
*/

#ifndef _LASER_ROTATE3D_CPP_
#define _LASER_ROTATE3D_CPP_

//system includes

#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <iostream>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

//LAR3 includes
#include <serialcom/SerialCom.h>

//Local includes
#include <laser_rotate_3d/laser_rotate3D.h>

//using namespace std;

//void RotateLaser::RotateLaserCallback(std::string* readdata)
// this function is called back once the readline thread is invoked
//void RotateLaserCallback(std::string* readdata)
//{
//		int thesize;
//	 	thesize = readdata->size();
//		// for now print on screen what has been read
//		printf("string size %s \n", readdata->c_str());//readdata->c_str);
//	
		// have to pass the information in order to safely allow publishing
	
//};

int main(int argc, char ** argv)
{

	ros::init(argc,argv,"RotateLaser");
	ros::Time::init(); // only to be used in testing and standalone.
	ros::Rate loop_rate(200); //it was 100
	
	//int baudrate = 9600;
	
	// instatiate the RotateLaser
	RotateLaser RotateTheLaser;

	/* assign a callback for reading the serial port: use binding because we are
	 	invoking a member function and boost does not support that directly
	http://www.boost.org/doc/libs/1_48_0/doc/html/function/tutorial.html
	http://www.boost.org/doc/libs/1_48_0/libs/bind/bind.html#with_member_pointers */
	RotateTheLaser.BoostRotateLaserCallback = std::bind1st(std::mem_fun(&RotateLaser::RotateLaserCallback), &RotateTheLaser); //&RotateLaserCallback;

	// has to read from the PIC via serial port and publish
	// start a thread for reading the serial port
	RotateTheLaser.ReadLinesSerialPort(RotateTheLaser.BoostRotateLaserCallback);
	

// 	while(ros::ok()) //this was uncomment
// 	{					//this was uncomment
	ros::spin();
// 		printf("this is %s \n", "test");
// 		RotateTheLaser.SendCommand();
// 		RotateTheLaser.Publish(); // this will publish the angle at the desired rate //this was uncomment
// 		ros::spinOnce(); 																//this was uncomment
// 		loop_rate.sleep();																//this was uncomment
// 	};
	
	return 0;
	
}

#endif //EOF