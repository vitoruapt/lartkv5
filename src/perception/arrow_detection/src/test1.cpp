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
 *	@file 	test1.cpp
 * 	@brief	sript that integrates Visp and OpenCV
 *  
 *	@author		César Sousa, cesarsousa@ua.pt
 *	@date 6-3-2014
 * 	@version V0.0
 *	@internal
 * 
 * 		Revision	---
 * 		Compiler	gcc
 * 		Company		DEM - Universidade de Aveiro
 * 		Copyright	Copyright (c) 2014, César Sousa
 * 
 * 		Info:		
 * 	
 *	command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

/*! \example tutorial-pose-from-points-tracking.cpp */

//c++
#include <stdio.h>
#include <iostream>

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

//convert images //cesar
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

//my files
#include <arrow_detection/findArrow.h>


void wait_lum_set(vpImage<unsigned char> &I_grey,
#if defined(VISP_HAVE_DC1394_2)
    vp1394TwoGrabber &g//THIS, is the one that is called!!! 
#elif defined(VISP_HAVE_CMU1394)
    vp1394CMUGrabber g
    
#endif
)
{
    std::cout << "**PLEASE SET THE CAMERA LUMINIOSITY!! ***" << std::endl;
    std::cout << "press 'Esc' to exit" << std::endl;
    while (true)
    {
        //if (cv::waitKey(20) == 27)
        if(vpDisplay::getClick(I_grey))
        {
            break;
        }
        g.acquire(I_grey);//g.acquire(I);
        vpDisplay::display(I_grey);//vpDisplay::display(I);
        vpDisplay::flush(I_grey);
    }
    std::cout << "Luminiosity setted!" << std::endl;
}

int main()
{
#if (defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394))
  try {
    vpImage<unsigned char> I; // Create a gray level image container

#if defined(VISP_HAVE_DC1394_2)
    vp1394TwoGrabber g(false);
#elif defined(VISP_HAVE_CMU1394)
    vp1394CMUGrabber g;
#endif
    g.open(I);
    g.acquire(I);

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Camera view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::display(I);
    vpDisplay::flush(I);
std::cout << "teste1" << std::endl;
    vpDot2 blob;
    blob.setGraphics(true);
    blob.setGraphicsThickness(2);
    //blob.initTracking(I);
    cv::Point src_centre;   bool init = true;   cv:Mat I_mat;
    std::cout << "teste2" << std::endl;
    
    wait_lum_set(I,g);//set luminiosity
    
    while (true)
    {
        try
        {
            while (init)
            {
                std::cout << "teste3" << std::endl;
                vpImageConvert::convert(I,I_mat);
                vpTime::wait(10);
                std::cout << "teste4" << std::endl;
                findArrow::find_arrow(I_mat,src_centre);
                if (findArrow::found_new_point == true)
                {
                    std::cout << "teste5" << std::endl;
                    //blob.initTracking(I,vpImagePoint(240, 320));//cesar
                    blob.initTracking(I,vpImagePoint(src_centre.y, src_centre.x));//cesar
                    init = false;
                }
                //update image
                g.acquire(I);
                vpDisplay::display(I);
                vpDisplay::flush(I);
            }
            std::cout << "teste6" << std::endl;
            while(1) 
            {
                std::cout << "teste7" << std::endl;
                std::cout << "getCog" << blob.getCog() << std::endl;
                
                g.acquire(I); // Acquire an image
                vpDisplay::display(I);
                blob.track(I);
                vpDisplay::flush(I);
                if (vpDisplay::getClick(I, false))
                break;
            }
        }
        catch(vpException e) 
        {
            std::cout << "Catch an exception: " << e << std::endl;
            init=true;
        }
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
