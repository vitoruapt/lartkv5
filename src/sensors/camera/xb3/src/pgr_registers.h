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
 * @addtogroup pgr
 *@{
 */
#ifndef PGR_REGISTERS_H
#define PGR_REGISTERS_H

/**
\file
\brief Header file for the registers for the PGR toolbox
*/

//=============================================================================
// Copyright © 2007 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research Inc.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================

//=============================================================================
//
// pgr_registers.h
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <dc1394/control.h>
#include <dc1394/conversions.h>

//=============================================================================
// PGR Includes
//=============================================================================

// PGR specific register that contains the Bayer Tile mapping information
#define BAYER_TILE_MAPPING_REGISTER 	(0x1040)
#define SENSOR_BOARD_INFO_REGISTER  	(0x1f28)
#define IMAGE_DATA_FORMAT_REGISTER	(0x1048)

//=============================================================================
// Name: getSensorInfo
//
// Input:
//  camera      - The camera to be queried
//
// Output:
//  pbColor   	- Is the camera color?
//  pnRows	- Number of rows in image resolution
//  pnCols	- Number of columns in image resolution
//
// Description:
//  This function queries the PGR registers that identify the sensor info.
//  This is a PGR specific register query.
//
//=============================================================================
dc1394error_t
getSensorInfo( dc1394camera_t* 	camera,
	       bool*		pbColor,
	       unsigned int*	pnRows,
	       unsigned int*	pnCols );

//=============================================================================
// Name: setEndian
//
// Input:
//  camera      - The camera to be queried
//
// Output:
//  bBigEndian  - "true" means transmit 16-bit data in big endian 
//		  (DCAM-compliant) mode
//		- "false" means transmit 16-bit data in little endian mode
//		  which allows the buffers to be directly cast into unsigned
//		  short on Intel-based PC platforms
//
// Description:
//
//=============================================================================
dc1394error_t
setEndian( dc1394camera_t* camera,
	   bool bBigEndian );

//=============================================================================
// Name: getBayerTile
//
// Input:
//  camera      - The camera to be queried
//
// Output:
//  bayerTile   - Returns with the Bayer tile pattern 
//
// Description:
//  This function queries the PGR registers that identify the bayer tile
//  pattern for a bayer camera.
//
//=============================================================================
dc1394error_t
getBayerTile( dc1394camera_t* camera,
	      dc1394color_filter_t* bayerPattern );

#endif
 /**
 *@}
 */
/*Previous 3 lines appended automatically on Wed Jun  9 00:11:56 WEST 2010 */
