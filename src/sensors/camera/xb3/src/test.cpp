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
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

/**
\file
\brief Just a test file for seeing if the xb3 module works
*/

#include <signal.h>
//=============================================================================
// PGR Includes
//=============================================================================
#include "pgr_registers.h"
#include "pgr_stereocam.h"

#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}

TriclopsContext wideTriclops;
TriclopsContext shortTriclops;

TriclopsError e;

char file_shortcal[1024] = "config/short.cal";
char file_widecal[1024]	= "config/wide.cal";



int main(int argc, char **argv)
{


struct{
	int width;
	int height;
}Image;
	Image.width=320;
	Image.height=240;


	// read in the TriclopsContexts from calibration files
	printf( "Getting TriclopsContexts from files \n%s \n and \n%s ... \n", file_shortcal, file_widecal);
	e = triclopsGetDefaultContextFromFile( &shortTriclops, file_shortcal);
	if ( e != TriclopsErrorOk ) { fprintf( stderr, "Can't get short context from file\n" ); }
	//triclopsWriteDefaultContextToFile( shortTriclops, (char*)"../config/short_context.txt");

	//e = triclopsGetDefaultContextFromFile( &wideTriclops,file_widecal); 
	//if ( e != TriclopsErrorOk ){	fprintf( stderr, "Can't get wide context from file\n" ); shutdown_module(SIGINT);}
	//triclopsWriteDefaultContextToFile( wideTriclops, (char*)"../config/wide_context.txt");
	// make sure we are in subpixel mode
	//triclopsSetSubpixelInterpolation( wideTriclops, 1 );

	PFLN
	triclopsSetMaxThreadCount( shortTriclops,1 );
	PFLN
	//triclopsSetMaxThreadCount( wideTriclops,1 );
	PFLN


	}
