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
// Copyright � 2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================
//=============================================================================
// $Id: triclopsbuffer.h,v 2.3 2007/11/30 00:43:03 soowei Exp $
//=============================================================================
#ifndef TRICLOPSBUFFER_H
#define TRICLOPSBUFFER_H

//=============================================================================
//
// This file defines the the API for Buffer management calls in the 
// Triclops Stereo Vision SDK.
//
//=============================================================================


//=============================================================================
// Defines
//=============================================================================

//=============================================================================
// System Includes  
//=============================================================================

#include <triclops.h>

#ifdef __cplusplus
extern "C"
{
#endif
   
//=============================================================================
// Macros  
//=============================================================================

//=============================================================================
// Enumerations  
//=============================================================================

//=============================================================================
// Types 
//=============================================================================

//=============================================================================
// Function Prototypes  
//=============================================================================

//=============================================================================
// Image Buffer Operations
//=============================================================================
//
// Group = Image Buffer Operations 

//
// Name: triclopsSetImageBuffer
//
// Synopsis:
//  Sets the internal image buffer for the specified camera and image type 
//  to be the buffer supplied by the user.
//
// Input:
//  context   - The context.
//  buffer    - A user allocated buffer of sufficient size.
//  imageType - The image type.
//  camera    - The camera.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidCamera  - The input camera is invalid for this camera configuration, 
//                  or is not associated with the requested image type.
//  InvalidRequest - The image is not set to be generated by the stereo kernel.
//
// Description:
//  This function allows the user to specify directly what memory he/she 
//  wishes the output images to be deposited into.  This memory will be used 
//  by the stereo  kernel as working space.  This has the advantage of saving 
//  a copy for tasks such as displaying to the screen.  The user may simply 
//  set the output image buffer to his/her display buffer.  However, since 
//  this memory will be used by the stereo kernel as working space, the 
//  contents of the buffer may change with each call of triclopsPreprocess()
//  or triclopsStereo().  If the results are to be saved, it is the user's 
//  responsibility to do so.  In addition, the user is responsible to allocate
//  sufficient memory for the buffer, and to de-allocate the buffer after 
//  it is no longer needed.  Before de-allocating the buffer, the user should 
//  call triclopsUnsetImageBuffer().  If the user requests an invalid image,
//  such as the disparity image, when the context stereo flag is set to false,
//  or the edge image when edge correlation is set to false, an error of 
//  invalid request will be returned. Disparity images are always associated
//  with the reference camera.
//
// See Also:
//  triclopsGetImage(), triclopsUnsetImageBuffer()
//
TriclopsError
triclopsSetImageBuffer( TriclopsContext	   context,
			unsigned char*	   buffer,
			TriclopsImageType  imageType,
			TriclopsCamera	   camera );

//
// Name: triclopsUnsetImageBuffer
//
// Synopsis:
//  This releases the user specified internal image buffer for the specified 
//  camera and image type.  The next time this buffer is required by the
//  system, it will allocate a new one for internal use.
//
// Input:
//  context   - The context.
//  imageType - The image type.
//  camera    - The camera.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidCamera  - The input camera is invalid for this camera configuration, 
//                  or is not associated with the requested image type.
//
// Description:
//  If the user has already called triclopsSetImageBuffer for a particular 
//  camera and image type, the stereo kernel will be using that buffer for 
//  internal processing.  If the user no longer wants to have the supplied 
//  buffer used by the stereo kernel, he/she may use this function to inform 
//  the stereo kernel that it is no longer available.  A new buffer will be 
//  created the next time it is required by the stereo kernel.
//
// See Also:
//  TriclopsGetImage(), triclopsSetImageBuffer()
//
TriclopsError
triclopsUnsetImageBuffer( TriclopsContext   context,
			  TriclopsImageType imageType,
			  TriclopsCamera    camera );


//
// Name: triclopsSetImage16Buffer
//
// Description:
//  This function allows the user to set the location to which 16bit 
//  (generally subpixel) depth images are written to once they are processed.
// 
// Input:
//  context   - The TriclopsContext to set the buffer in.
//  buffer    - A pointer to the buffer.
//  imageType - The type of image to be written to the buffer.
//  camera    - The camera to write from.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
// 
TriclopsError
triclopsSetImage16Buffer( TriclopsContext      	context,
			  unsigned short*      	buffer,
			  TriclopsImage16Type  	imageType,
			  TriclopsCamera	camera );

//
// Name: triclopsUnsetImage16Buffer
//
// Synopsis:
//  This releases the user specified internal image buffer for the specified 
//  camera and image type.  The next time this buffer is required by the
//  system, it will allocate a new one for internal use.
//
// Input:
//  context   - The context.
//  imageType - The image type.
//  camera    - The camera.	
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext  - The input context was invalid.
//  InvalidCamera   - The input camera is invalid for this camera configuration, 
//                  or is not associated with the requested image type.
//
// Description:
//  If the user has already called triclopsSetImage16Buffer for a particular 
//  camera and image type, the stereo kernel will be using that buffer for 
//  internal processing.  If the user no longer wants to have the supplied 
//  buffer used by the stereo kernel, he/she may use this function to inform 
//  the stereo kernel that it is no longer available.  A new buffer will be 
//  created the next time it is required by the stereo kernel.
//
// See Also:
//  triclopsGetImage(), triclopsSetImage16Buffer()
//
TriclopsError
triclopsUnsetImage16Buffer( TriclopsContext     context,
			    TriclopsImage16Type imageType,
			    TriclopsCamera      camera );

//
// Name: triclopsSetColorImageBuffer
//
// Synopsis:
//  Allows the user to set separate buffers to which individual bands of the
//  processed color image are written to.  In this case, the "processed" image
//  means the rectified image that is rectified when triclopsRectifyColorImage()
//  is called.
//
// Input:
//  context - The TriclopsContext to set the buffer for.
//  nCamera - The camera buffer to set.
//  red     - A pointer to the red buffer.
//  green   - A pointer to the green buffer.
//  blue    - A pointer to the blue buffer.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
//
// See Also:
//  triclopsRectifyColorImage()
//
TriclopsError
triclopsSetColorImageBuffer( TriclopsContext  context,
			     TriclopsCamera   nCamera,
			     unsigned char*   red,
			     unsigned char*   green,
			     unsigned char*   blue );

//
// Name: triclopsUnsetColorImageBuffer
//
// Synopsis:
//  This releases the user specified internal color image buffer for the 
//  specified camera.  The next time this buffer is required by the
//  system, it will allocate a new one for internal use.
//
// Input:
//  context   - The context.
//  camera    - The camera.	
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext  - The input context was invalid.
//  InvalidCamera   - The input camera is invalid for this camera configuration, 
//                  or is not associated with the requested image type.
//
// Description:
//  If the user has already called triclopsSetColorImageBuffer for a particular 
//  camera, the stereo kernel will be using that buffer when rectifying a
//  color image. If the user no longer wants to have the supplied 
//  buffer used by the stereo kernel, he/she may use this function to inform 
//  the stereo kernel that it is no longer available.  A new buffer will be 
//  created the next time it is required by the stereo kernel.
//
// See Also:
//  triclopsRectifyColorImage(), triclopsSetColorImageBuffer()
//
TriclopsError
triclopsUnsetColorImageBuffer( TriclopsContext   context,
			       TriclopsCamera    camera );

//
// Name: triclopsSetPackedColorImageBuffer
//
// Synopsis: 
//  Allows the user to set the buffer to which the processed color image
//  is written to.
//
// Input:
//  context - The TriclopsContext to set the buffer for.
//  nCamera - The camera buffer to set.
//  buffer  - A pointer to a buffer of TriclopsPackedColorPixels.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
//
TriclopsError
triclopsSetPackedColorImageBuffer( TriclopsContext	     context,
				   TriclopsCamera	     nCamera,
				   TriclopsPackedColorPixel* buffer );

//
// Name: triclopsUnsetPackedColorImageBuffer
//
// Synopsis:
//  This releases the user specified internal color image buffer for the 
//  specified camera.  The next time this buffer is required by the
//  system, it will allocate a new one for internal use.
//
// Input:
//  context   - The context.
//  camera    - The camera.	
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext  - The input context was invalid.
//  InvalidCamera   - The input camera is invalid for this camera configuration, 
//                  or is not associated with the requested image type.
//
// Description:
//  If the user has already called triclopsSetPackedColorImageBuffer for a 
//  particular  camera, the stereo kernel will be using that buffer when rectifying a
//  color image. If the user no longer wants to have the supplied 
//  buffer used by the stereo kernel, he/she may use this function to inform 
//  the stereo kernel that it is no longer available.  A new buffer will be 
//  created the next time it is required by the stereo kernel.
//
// See Also:
//  triclopsSetPackedColorImageBuffer(), triclopsRectifyPackedColorImage()
//
TriclopsError
triclopsUnsetPackedColorImageBuffer( TriclopsContext   context,
				     TriclopsCamera    camera );

#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPSBUFFER_H
