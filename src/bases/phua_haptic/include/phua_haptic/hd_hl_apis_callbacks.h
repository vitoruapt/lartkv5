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
 * @brief hd_hl_apis_callbacks.h file for this module. Contains includes, prototypes and global vars.
 * @author pedro_cruz
 * @version 1.0
 * @date 17 May 2012
 *@{
 */
#ifndef _HD_HL_APIS_CALLBACKS_H
#define _HD_HL_APIS_CALLBACKS_H

/*~~~~~~~~~~~~~~~~~ 
|| OpenHaptics 3  ||
~~~~~~~~~~~~~~~~~~*/
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HL/hl.h>
#include <HLU/hlu.h>
#include <HDU/hdu.h>
#include <HDU/hduMath.h>

/*~~~~~~~~~~~~~~~~~ 
||     others     ||
~~~~~~~~~~~~~~~~~~*/
#include <stdlib.h>
#include <stdio.h>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <phua_haptic/types.h>
#include <phua_haptic/miscellaneous.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HL/hl.h>

#include <armadillo>

#include <phua_haptic/hd_hl_apis_aux.h>

#include <phua_haptic/haptic_rendering_funx.h>

//apparent mass at tooltip
#define PHANToM_TOOLTIP_MASS 0.047

/** 
* @brief Callback for max priority servo loop update.

  This function is the max priority callback that runs in the servo loop thread.
It updates the data structure with the current joystick values.
* @param pUserData a pointer to a structure.
* @return HD_CALLBACK_CONTINUE.
*/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData);

/** 
* @brief Callback for retrieving device specific parameters.

  This function is a callback executed once in the code that retrieves nominal maxmimum stiffness,
damping, force and continuous force, specific for each device.
* @param pUserData a pointer to a structure.
* @return HD_CALLBACK_CONTINUE.
*/
HDCallbackCode HDCALLBACK updateDeviceParametersCallback(void *pUserData);

/** 
* @brief Callback to update device data structure with the relevant parameters.

  This function is a callback that runs whenever the haptic loop starts and updates
the device data structure with the relevant parameters chosen by the user.
* @param pUserData a pointer to a structure.
* @return HD_CALLBACK_CONTINUE.
*/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData);

/** 
* @brief Callback to update the force feedback of the joystick.

  This function is a callback that runs within the haptic loop.
  It checks the selected kinematic model and applys force whenever
the chosen end point leaves the robot workspace.
* @param pUserData a pointer to a structure.
* @return HD_CALLBACK_CONTINUE.
*/
HDCallbackCode HDCALLBACK forcefeedbackCallback(void *pUserData);

/** 
* @brief Callback for device calibration.

This function is PHANToM OMNI specific, for each device can have a different calibration method.
* @param pUserData a pointer to a structure.
* @return HD_CALLBACK_CONTINUE.
*/
HDCallbackCode HDCALLBACK CalibrationCallback(void *pUserData);

#endif
/**
 *@}
*/
