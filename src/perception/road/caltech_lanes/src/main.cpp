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
 * \file main.cpp
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date Wed Oct 6, 2010
 *
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "ProcessImage.h"

#include "main.hh"
#include "cmdline.h"
#include "LaneDetector.hh"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>


namespace enc = sensor_msgs::image_encodings;

// Useful message macro
#define MSG(fmt, ...) \
  (fprintf(stdout, "%s:%d msg   " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? 0 : 0)

// Useful error macro
#define ERROR(fmt, ...) \
  (fprintf(stderr, "%s:%d error " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : -1)
using namespace std;
using namespace cv;


Procecess pross;

/**
 *  intersept Ctrt+C handler
 */
void sighandler(int sig)
{
	ROS_ERROR("Signal %d caught...",sig);
	cout<<"Shuting down road_recognition"<<endl;
	exit(0);
}


namespace LaneDetector
{
/**
 *  callback quando tem uma mensagem nova
 */
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	
	cv_bridge::CvImagePtr cv_ptr;
	
	try
	{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Caltech::main.cpp::cv_bridge exception:%s",e.what());
		exit(0);
	}
	
	
	// elapsed time
	clock_t elapsed = 0;
	
	pross.ProcessImage( cv_ptr->image, NULL, 0,&elapsed);
	double elapsedTime = static_cast<double>(elapsed) / CLOCKS_PER_SEC;	
	MSG("Total time %f secs for 1 image = %f Hz", elapsedTime,
	    1. / elapsedTime);
	cv::waitKey(10);
}
}
/**
 *  rotina inicial do meu programa
 */
int main(int argc, char** argv)
{

	ros::init(argc, argv, "caltech_lane");
	// carregar as definições para a class
	pross.Load_config( argc , argv );
	
	ros::NodeHandle nh;
	signal(SIGINT, &sighandler);
	
	/* load all the paramters of the camera */
// 	pross.Load_config(argc, argv);
	
	/* Create an ImageTransport instance, initializing it with our NodeHandle. */
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
// 
// 	
	ros::spin();
	ROS_INFO("Caltech::main.cpp::No error.");
}
