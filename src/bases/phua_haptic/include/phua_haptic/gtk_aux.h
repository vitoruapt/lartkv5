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
 * @brief gtk_aux.h file for this module. Contains prototypes and includes.
 * @author pedro_cruz
 * @version 1.0
 * @date 16 May 2012
 *@{
 */
#ifndef _GTK_AUX_H_
#define _GTK_AUX_H_

#include <gtk/gtk.h>
#include <glib.h>
#include <phua_haptic/types.h>
#include <boost/format.hpp>
#include <ros/package.h>
#include <iostream>

#include <armadillo>

#include <phua_haptic/humanoid_control_functions.h>

#define POINTS_FILE_NAME_STRING "demo_user_path_points.txt"

#define RIGHT_ARM_SAVE_STRING 	"ARM_RIGHT "
#define LEFT_ARM_SAVE_STRING 	"ARM_LEFT "
#define BOTH_ARMS_SAVE_STRING 	"ARM_BOTH "
#define LEFT_D_LEG_SAVE_STRING 	"D_LEG_LEFT "
#define RIGHT_D_LEG_SAVE_STRING	"D_LEG_RIGHT "

/** 
* @brief Function to update the main page joint value labels.

This function updates the main page joint value labels with the current values.
* @param RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateLabels(shared_vars_t*RobotVars);

/** 
* @brief Function to update the main page force value bars.

This function updates the main page progress bars with the current exerted force value.
* @param RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateForceBars(shared_vars_t*RobotVars);

/** 
* @brief Function to update the status bar.

This function updates the status bar with the calculation frequencies of the program.
* @param RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateStatusBar(shared_vars_t*RobotVars);

/** 
* @brief Function to remove minus signal from float, double or long double variables.

* @param value value to assess.
* @return assessed value.
*/
template <typename Type>
Type AvoidMinusZero(Type value)
{
  return (value == (Type)0 ? 0 : value);
}

#endif
/**
 *@}
*/
