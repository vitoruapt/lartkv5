/*******************************************************************************
*******************
  Software License Agreement (BSD License)
 
  Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - 
http://lars.mec.ua.pt
  All rights reserved.
 
  Redistribution and use in source and binary forms, with or without 
modification, are permitted
  provided that the following conditions are met:
 
   *Redistributions of source code must retain the above copyright notice, this 
list of
    conditions and the following disclaimer.
   *Redistributions in binary form must reproduce the above copyright notice, 
this list of
    conditions and the following disclaimer in the documentation and/or other 
materials provided
    with the distribution.
   *Neither the name of the University of Aveiro nor the names of its 
contributors may be used to
    endorse or promote products derived from this software without specific 
prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
DAMAGE.
 
********************************************************************************
*******************/

/*! 
 *      @file   rotateImage.h
 *      @brief  this is the header that includes the necessary libraries to use 
in the main body of the class and declares for the first time all the functions 
and variables existing in that same class. The purpose of this class is to 
subscribe to a ROS node, obtain a frame and finally, rotate and warp that same 
frame accordingly.
 *  
 *      @author        João Peixoto, joao.peixoto@ua.pt
 *      @date 17/03/2015
 *      @version V0.0
 *      @internal
 * 
 *              Revision        ---
 *              Compiler        gcc
 *              Company         DEM - Universidade de Aveiro
 *              Copyright       Copyright (c) 2015, João Peixoto
 * 
 *              Info:           
 *      
 *      command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */


#ifndef _ARROW_DETECTION_H_
#define _ARROW_DETECTION_H_

#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>

//ROS - this library will allow the usage of ROS, subscription of topics and 
//image conversion
#include <ros/ros.h>
#include <image_transport/image_transport.h>

//OpenCV - those libraries will allow the rotation and wrapping of the frame
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

//namespaces
using namespace std;
using namespace cv;

//created headers (LAR CREATED)
#include <imu_network/sensors_network.h>
#include <imu_network/filtered_imu_network.h>
#include <imu_network/filtered_imu.h>

// Global variables
string WINDOW = "Corrected Image [Press q in the image to close it]";
#define PI 3.14159265358979323846
bool show_math_data_G;                   //prints (or not) the angles in the terminal
bool show_image_G;                       //shows (or not) the image obtained in OpenCv

//Declaration of functions
void setBools(void);

#endif









/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~~~~~~~~~~ IMPORTANT INFORMATION REGARDING SENSORS  ~~~~~~~~~~
~~~~~~~~~~ AND CAMERA USAGE                         ~~~~~~~~~~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TO BE ABLE TO ALLOW THE USAGE OF THE PORT!
sudo adduser $USER dialout


 */