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
 * @file
 * @brief This is from point grey. Check the manual for instructions.
 */

//=============================================================================
// Copyright © 2000 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: pnmutils.h,v 1.14 2008/03/06 23:04:06 donm Exp $
//=============================================================================
#ifndef _PNMUTILS_H_
#define _PNMUTILS_H_


//=============================================================================
//
// pnmutils:
//
// This file provides functions for reading and writing a PGM and PPM
// files from and to a TriclopsImage.
// Users are encouraged to modify this source to suit their own needs.
//=============================================================================


//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"


#ifdef __cplusplus
extern "C"
{
#endif


//=============================================================================
// pgmReadToTriclopsImage()
//
//	This function reads an 8-bit pgm file into a TriclopsImage 
// 	structure.  It allocates the data within the TriclopsImage to contain
//	the image data.  This data must be freed after it is used by calling
//	'freeImage()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
pgmReadToTriclopsImage(	const char* 	filename,
			TriclopsImage*	image );

//=============================================================================
// pgmReadToTriclopsImage16()
//
//	This function reads an 16-bit pgm file into a TriclopsImage16
// 	structure.  It allocates the data within the TriclopsImage16 to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeImage16()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
pgmReadToTriclopsImage16(  const char* 	     filename,
			   TriclopsImage16*  image );


//=============================================================================
//
//
//
//
TriclopsBool
pgmRead3ToTriclopsInput(   const char*	  redFileName,
			   const char*	  bluFileName,
			   const char*	  greFileName,
			   TriclopsInput* pInput );

//=============================================================================
// pgmReadToTriclopsInput()
//
//	This function reads a pgm file into a TriclopsInput
// 	structure (of type TriInp_RGB).  
//	It allocates the data within the TriclopsInput to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeInput()'
//
//	This data is suitable to be be input into the Triclops stereo
//	library with a call to 'triclopsPreprocess()'
//
//	NOTE: this uses a greyscale image and expects that the red/green
//	or red/green/blue channels be side by side or "row-interleaved".
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
pgmReadToTriclopsInput( const char*	filename,
			TriclopsInput*	input );


//=============================================================================
// pgmReadToTriclopsInput()
//
//	This function reads a pgm file into a TriclopsInput
// 	structure (of type TriInp_RGB).  
//	It allocates the data within the TriclopsInput to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeInput()'
//
//	This data is suitable to be be input into the Triclops stereo
//	library with a call to 'triclopsPreprocess()'
//
//	NOTE: this uses a greyscale image and expects that the red/green
//	or red/green/blue channels be side by side or "row-interleaved".
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
pgmReadToTriclopsInput( const char*	filename,
			TriclopsInput*	input );


//=============================================================================
// ppmReadToTriclopsInput()
//
//	This function reads a ppm file into a TriclopsInput
// 	structure (of type TriInp_RGB_32BITPACKED).  
//	It allocates the data within the TriclopsInput to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeInput()'
//
//	This data is suitable to be be input into the Triclops stereo
//	library with a call to 'triclopsPreprocess()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
ppmReadToTriclopsInput(	const char*    filename,
			TriclopsInput* input );


//=============================================================================
// ppmReadToTriclopsInputRGB()
//
//	This function reads a ppm file into a TriclopsInput
// 	structure (of type TriInp_RGB).  
//	It allocates the data within the TriclopsInput to 
//	contain the image data.  This data must be freed after it is used 
//	by calling 'freeInput()'
//
//	This data is suitable to be be input into the Triclops stereo
//	library with a call to 'triclopsPreprocess()'
//
// return codes:
//	True	- the file was successfully read and memory allocated
//	False	- the file read failed (no memory was allocated)
//
TriclopsBool
ppmReadToTriclopsInputRGB(	const char*    filename,
				TriclopsInput* input );


//=============================================================================
// freeInput()
//
//	This function frees the image memory associated with a TriclopsInput.
//	This needs to be called after the user is finished with a 
//	TriclopsInput structure that was created from a called to
// 	'ppmReadToTriclopsInput()'
//
// return codes:
//	True	- the memory was freed
//
TriclopsBool
freeInput( TriclopsInput* input );


//=============================================================================
// freeImage()
//
//	This function frees the image memory associated with a TriclopsImage.
//	This needs to be called after the user is finished with a 
//	TriclopsImage structure that was created from a called to
// 	'pgmReadToTriclopsImage()'
//
// return codes:
//	True	- the memory was freed
//
TriclopsBool
freeImage( TriclopsImage* pimage );


//=============================================================================
// freeImage16()
//
//	This function frees the image memory associated with a 
//	TriclopsImage16.
//	This needs to be called after the user is finished with a 
//	TriclopsImage structure that was created from a called to
// 	'pgmReadToTriclopsImage16()'
//
// return codes:
//	True	- the memory was freed
//
TriclopsBool
freeImage16( TriclopsImage16* pimage );


//=============================================================================
// pgmWriteFromTriclopsImage()
//
//	This function writes an 8-bit pgm file from a TriclopsImage 
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
pgmWriteFromTriclopsImage( const char* 	     filename,
			   TriclopsImage*    image );


//=============================================================================
// ppmWriteFromTriclopsColorImage()
//
//	This function writes an 24-bit ppm file from a TriclopsColorImage
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
ppmWriteFromTriclopsColorImage(	 const char*	      filename,
				 TriclopsColorImage*  image );


//=============================================================================
// pgmWrite3FromTriclopsInput()
//
//      This function writes 3 8-bit pgm files from a TriclopsInput structure.
//      These correspond to the appropriate channels of the image.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
pgmWrite3FromTriclopsInput(   const char*    redFilename,
			      const char*    greFilename,
			      const char*    bluFilename,
			      TriclopsInput* input );


//=============================================================================
// pgmWrite3FromTriclopsInputWithComment()
//
//      This function writes 3 8-bit pgm files from a TriclopsInput structure.
//      These correspond to the appropriate channels of the image.  The comment
//      will be stored in each file (pass in NULL if no comment is desired).
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
pgmWrite3FromTriclopsInputWithComment( const char*    redFilename,
				       const char*    greFilename,
				       const char*    bluFilename,
				       const char*    comment,
				       TriclopsInput* input );


//=============================================================================
// pgmWriteFromTriclopsImage16()
//
//	This function writes an 16-bit pgm file from a TriclopsImage 
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//
TriclopsBool
pgmWriteFromTriclopsImage16(  const char*	filename,
			      TriclopsImage16*	image );


//=============================================================================
// ppmWriteFromTriclopsInput()
//
//	This function writes an 24-bit ppm file from a TriclopsInput 
//	structure.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//		- or the TriclopsInput was not in a supported format
//
TriclopsBool
ppmWriteFromTriclopsInput( const char*	  filename,
			   TriclopsInput* input );


//=============================================================================
//
//
//
//
TriclopsBool
ppmWriteFromTriclopsInputWithComment(  const char*    filename,
				       const char*    comment,
				       TriclopsInput* input );

//=============================================================================
// pgmWriteFromTriclopsInput()
//
//	This function writes an 24-bit ppm file from a TriclopsInput 
//	structure.
//
//	nCameras - if it is known that the Input has only 2 cameras,
//		this can be changed to a value of 2. Otherwise it should
// 		be 3
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//		- or the TriclopsInput was not in a supported format
//
TriclopsBool
pgmWriteFromTriclopsInput( const char*	  	filename,
			   TriclopsInput* 	input,
			   int 			nCameras );


//=============================================================================
//
//
//
//
TriclopsBool
pgmWriteFromTriclopsInputWithComment(  const char*    	filename,
				       const char*    	comment,
				       TriclopsInput* 	input,
				       int 		nCameras = 3 );



//=============================================================================
// pgmWriteFromTriclopsInput()
//
//	This function writes an 8-bit pgm file from a TriclopsInput 
//	structure.
//
//	nCameras - if it is known that the Input has only 2 cameras,
//		this can be changed to a value of 2.  Otherwise it should
//		be 3.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//		- or the TriclopsInput was not in a supported format
//
TriclopsBool
pgmWriteFromTriclopsInput( const char*	  	filename,
			   TriclopsInput* 	input,
			   int 			nCameras );


//=============================================================================
// pgmWriteFromTriclopsInputWithComment()
//
//	This function writes an 8-bit pgm file from a TriclopsInput 
//	structure.  The comment may be NULL.
//
//	nCameras - if it is known that the Input has only 2 cameras,
//		this can be changed to a value of 2. Otherwise it should
//		be 3.
//
// return codes:
//	True	- the file was successfully written
//	False	- 'filename' could not be opened for writing
//		- or the TriclopsInput was not in a supported format
//
TriclopsBool
pgmWriteFromTriclopsInputWithComment(  const char*    	filename,
				       const char*    	comment,
				       TriclopsInput* 	input,
				       int 		nCameras );


#ifdef __cplusplus
}
#endif


#endif //#ifndef _PNMUTILS_H_
