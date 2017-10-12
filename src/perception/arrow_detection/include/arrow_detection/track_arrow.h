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

/*! 
 *  @file   trak_arrow.cpp
 *  @brief  using VISP trak, it will track the point from "/find_arrow_position" topic and serd the comand for servos.
 *  
 *  @author     César Sousa, cesarsousa@ua.pt
 *  @date       21-10-2014
 *  @version    V0.0
 *  @internal
 * 
 *      Revision    ---
 *      Compiler    gcc
 *      Company     DEM - Universidade de Aveiro
 *      Copyright   Copyright (c) 2014, César Sousa
 * 
 *      Info:       
 *  
 *  command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

#ifndef TRACK_ARROW_H
#define TRACK_ARROW_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <visp_ros/vpROSGrabber.h>
//Visp
#include <visp/vpPixelMeterConversion.h>
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpPose.h>
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

#include <fstream>
#include <iostream>

#include <unistd.h>

#include <stack>
#include <ctime>

using namespace std;

geometry_msgs::Point GLOBAL_POINT; //point that will be updated wen arrow detection send the new point

bool receive_new_point = false;

#endif
