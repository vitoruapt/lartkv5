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
 * @addtogroup xb3
 *@{
 */

/**
\file
\brief Header file for the test file
*/

#ifndef _XB3_H_
#define _XB3_H_

//####################################################################
// Includes:
//####################################################################

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <signal.h>
//=============================================================================
// PGR Includes
//=============================================================================
#include "pgr_registers.h"
#include "pgr_stereocam.h"

#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}



typedef struct{
	 int use_shm;
	char dummymode;
	char debugmode;
}TYPE_flg;





void handler_short_stereomask_change(char *a, char *b, char *c);
void handler_wide_stereomask_change(char *a, char *b, char *c);
int convert_triclopscolorimage_2_iplimage(TriclopsColorImage *TI, IplImage *IPL);




#ifdef _XB3_CPP_


IplImage *image,*uimage,*pimage;
double filltime; //image timestamp

TriclopsInput  colorInput;
unsigned char* pucRightRGB		= NULL;
unsigned char* pucLeftRGB		= NULL;
unsigned char* pucCenterRGB		= NULL;

TriclopsError e;
TriclopsImage16 depthImage16;
unsigned char* pucGreenBuffer;
unsigned char* pucRGBBuffer;
unsigned char* pucDeInterlacedBuffer;
PGRStereoCamera_t stereoCamera;
dc1394camera_t* 	camera;
dc1394error_t 	err;

dc1394_t * d;
dc1394camera_list_t * list;
unsigned int nThisCam;

char file_shortcal[255] = "config/short.cal";
char file_widecal[255]	= "config/wide.cal";


//xb3_3dpointcloud_message pc_msg;
//xb3_3dpointcloud_message pc_msg1;

TriclopsColorImage shortRectifiedColor;
TriclopsColorImage wideRectifiedColor;





IplImage* RightCamera;
IplImage* CenterCamera;
IplImage* LeftCamera;
IplImage *Ileft;
IplImage *Iright;
IplImage *InvalidStereoMask;
IplImage *DicardedStereoMask;
IplImage *ShortDisparityImage;
IplImage *WideDisparityImage;


#endif

#endif
/**
*@}
*/
