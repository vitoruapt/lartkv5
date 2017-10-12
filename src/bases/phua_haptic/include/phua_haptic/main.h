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
/** @file
 * @brief main.h file file for this module. Contains includes.
 * @author pedro_cruz
 * @version 2.0
 * @date 7 May 2012
 *@{
 */
#ifndef __MAIN_H_
#define __MAIN_H_

/*~~~~~~~~~~~~~~~~~ 
|| base includes  ||
~~~~~~~~~~~~~~~~~~*/
#include <iostream>
#include <pthread.h>
#include <string.h>
#include <ctime>
#include <math.h>
#include <stdio.h>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

/*~~~~~~~~~~~~~~~~~ 
||  my includes   ||
~~~~~~~~~~~~~~~~~~*/

#include <phua_haptic/gtk_interface.h>
#include <phua_haptic/miscellaneous.h>
#include <phua_haptic/types.h>
#include <phua_haptic/hd_hl_apis_callbacks.h>
#include <phua_haptic/humanoid_control_functions.h>

/*~~~~~~~~~~~~~~~~~ 
|| OpenHaptics 3  ||
~~~~~~~~~~~~~~~~~~*/
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HL/hl.h>

/** 
* @brief Function to output ROS messages with PHANToM frames and calculate the pen tip position.
* @param[in] br the ROS transform broadcaster.
* @param[in] listener the ROS transform listener.
* @param[in] pUserData pointer to shared structure.
* @return none.
*/
void ROS_CalculatePHANToMPenFrame(tf::TransformBroadcaster *br, tf::TransformListener *listener, void *pUserData);

/** 
* @brief Function to output ROS marker vector with the demonstration's constructions.
* @param[in] pUserData pointer to shared structure
* @param[out] marker_vector marker array to hold the markers.
* @return none.
*/
void ROS_UpdateMarkers(std::vector<visualization_msgs::Marker>& marker_vector, void *pUserData);

#endif
/**
 *@}
*/
