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
 * @addtogroup distortion_correction 
 *@{
 * @file 
 * @brief Distortion correction class definition
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-09-15
 */
#ifndef _DISTORTION_CORRECTION_H_
#define _DISTORTION_CORRECTION_H_


//####################################################################
//// Includes:
////####################################################################
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <mit_darpa_logs_player/camtrans.h>


#include "camera_parameters.h"

class class_distortion_correction: virtual public class_camera_parameters
{
	// ------------------------------------------------------------------------------
	//PUBLIC SECTION OF THE CLASS
	// ------------------------------------------------------------------------------
	public:
		class_distortion_correction(void){};
		~class_distortion_correction(void){};

		int correct_image_distortion(const cv::Mat* distorted, cv::Mat* corrected)
		{
			cv::Mat K;
			K.create(3,3,CV_64F);   

			K.at<double>(0,0) = intrinsic.at<double>(0,0); 
			K.at<double>(0,1) = intrinsic.at<double>(0,1); 
			K.at<double>(0,2) = intrinsic.at<double>(0,2); 

			K.at<double>(1,0) = intrinsic.at<double>(1,0); 
			K.at<double>(1,1) = intrinsic.at<double>(1,1); 
			K.at<double>(1,2) = intrinsic.at<double>(1,2); 

			K.at<double>(2,0) = intrinsic.at<double>(2,0); 
			K.at<double>(2,1) = intrinsic.at<double>(2,1); 
			K.at<double>(2,2) = intrinsic.at<double>(2,2); 

			cv::undistort(*distorted, *corrected, K, distortion);
			return 1;
		}



		int correct_spherical_image_distortion(const cv::Mat* distorted, cv::Mat* corrected)
		{

			double position[3];                                                          
			double orientation_quat[4];

			CamTrans* cam = camtrans_new_spherical(spherical_distortion_params.calibration_width,
												   spherical_distortion_params.calibration_height,
													intrinsic.at<double>(0,0)/*fx*/,
													intrinsic.at<double>(1,1)/*fy*/,
													intrinsic.at<double>(0,2)/*intrinsic.cx*/,
													intrinsic.at<double>(1,2)/*intrinsic.cy*/,
													0 /*skew*/,
													position, orientation_quat, 
													spherical_distortion_params.cx,
													spherical_distortion_params.cy,
													spherical_distortion_params.param);

			camtrans_scale_image(cam, spherical_distortion_params.scale);
			cv::Mat mapx; 
			mapx.create(240,376,CV_32FC1); 

			cv::Mat mapy; 
			mapy.create(240,376,CV_32FC1); 

			for (double x=0; x<240; x++)
				for (double y=0; y<376; y++)
				{
					double ox,oy;
					//camtrans_distort_pixel(cam, x,y,&ox, &oy);          
					camtrans_distort_pixel(cam, y,x,&oy, &ox);          

					mapx.at<float>(x,y) = oy;
					mapy.at<float>(x,y) = ox;
				}

			cv::remap(*distorted, *corrected, mapx, mapy,CV_INTER_LINEAR);
			return 1;
		}
	private:

};

#endif
/**
 *@}
 */
