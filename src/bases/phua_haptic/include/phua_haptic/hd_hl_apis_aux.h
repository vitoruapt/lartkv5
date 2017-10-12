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
 * @brief hd_hl_apis_aux.h file for this module. Contains includes, prototypes and global vars.
 * @author pedro_cruz
 * @version 1.0
 * @date 10 July 2012
 *@{
 */
#ifndef _HD_HL_APIS_AUX_H
#define _HD_HL_APIS_AUX_H

/*~~~~~~~~~~~~~~~~~ 
|| OpenHaptics 3  ||
~~~~~~~~~~~~~~~~~~*/
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HL/hl.h>

#include <phua_haptic/types.h>
#include <phua_haptic/miscellaneous.h>

/*~~~~~~~~~~~~~~~~~ 
|| AUX FUNCTIONS  ||
~~~~~~~~~~~~~~~~~~*/

/** 
* @brief Function to rotate world oriented vector to PHANToM referencial.
* @param source vector to rotate.
* @return Rotated vector.
*/
hduVector3Dd TransformWorldCoordinatesToPHaNToMCoordinates(hduVector3Dd source);

/** 
* @brief Function to rotate PhANToM oriented vector to world referencial.
* @param source vector to rotate.
* @return Rotated vector.
*/
hduVector3Dd TransformPHANToMCoordinatesToWorldCoordinates(hduVector3Dd source);

/** 
* @brief Function to rotate PhANToM transform matrix to world referencial.
* @param source matrix to transform.
* @return transformed matrix.
*/
hduMatrix TransformPHANToMCoordinateMatrixToWorldCoordinateMatrix(hduMatrix source);

#endif
/**
 *@}
*/
