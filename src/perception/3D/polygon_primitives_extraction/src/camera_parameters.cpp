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
 * @addtogroup camera_parameters
 * @{
 * @file 
 * @brief Defines the camera parameters for distortion correction
 */

#ifndef _CAMERA_PARAMETERS_CPP_
#define _CAMERA_PARAMETERS_CPP_

#include "camera_parameters.h"

/**
 * @brief sets intrinsic parameters from a 3x3 CvMat
 *
 * @param fx the fx scaling parameter
 * @param fy the fy scaling parameter
 * @param cx the principal point x coordinates
 * @param cy the principal point y coordinates
 */
int class_camera_parameters::set_intrinsic(double fx, double fy, double cx, double cy)
{
	cv::Mat *M = &intrinsic;
	M->at<double>(0,0) = fx; M->at<double>(0,1) = 0; M->at<double>(0,2) = cx; M->at<double>(0,3) = 0;
	M->at<double>(1,0) = 0; M->at<double>(1,1) = fy; M->at<double>(1,2) = cy; M->at<double>(1,3) = 0;
	M->at<double>(2,0) = 0;	M->at<double>(2,1) = 0;  M->at<double>(2,2) = 1;  M->at<double>(2,3) = 0;

	return 1;};

/**
 * @brief 
 */
int class_camera_parameters::set_extrinsic(tf::StampedTransform* t)
{
	camera_6dof_position.setRotation(t->getRotation());
	camera_6dof_position.setOrigin(t->getOrigin());
	tf::Matrix3x3 rot(t->getRotation());
	set_extrinsic(rot[0][0], rot[0][1], rot[0][2], t->getOrigin().x(),
			rot[1][0], rot[1][1], rot[1][2], t->getOrigin().y(),
			rot[2][0], rot[2][1], rot[2][2], t->getOrigin().z());
	return 1;}

int class_camera_parameters::set_extrinsic(tf::Transform* t)
{
	camera_6dof_position.setRotation(t->getRotation());
	camera_6dof_position.setOrigin(t->getOrigin());
	tf::Matrix3x3 rot(t->getRotation());
	set_extrinsic(rot[0][0], rot[0][1], rot[0][2], t->getOrigin().x(),
			rot[1][0], rot[1][1], rot[1][2], t->getOrigin().y(),
			rot[2][0], rot[2][1], rot[2][2], t->getOrigin().z());
	return 1;}


/**
* @brief sets extrinsic parameters
*
* @param p11 pij the line i, coordinate j matrix parameter.
*/
int class_camera_parameters::set_extrinsic(double p11, double p12, double p13, double p14,
		double p21, double p22, double p23, double p24,
		double p31, double p32, double p33, double p34)
{
	cv::Mat *M = &extrinsic;
	M->at<double>(0,0) = p11; M->at<double>(0,1) = p12; M->at<double>(0,2) = p13; M->at<double>(0,3) = p14;
	M->at<double>(1,0) = p21; M->at<double>(1,1) = p22; M->at<double>(1,2) = p23; M->at<double>(1,3) = p24;
	M->at<double>(2,0) = p31; M->at<double>(2,1) = p32; M->at<double>(2,2) = p33; M->at<double>(2,3) = p34;
	M->at<double>(3,0) = 0;   M->at<double>(3,1) = 0;   M->at<double>(3,2) = 0;   M->at<double>(3,3) = 1;

	extrinsic_inverse = extrinsic.inv();
	return 1;};



#endif
/**
 *@}
 */      
