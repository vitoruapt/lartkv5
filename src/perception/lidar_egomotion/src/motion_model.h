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
\file
\brief Nonholonomic motion model declaration
*/

#ifndef _MOTION_MODEL_H_
#define _MOTION_MODEL_H_

#include <kfilter/ekfilter.hpp>
#include <string.h>
#include <math.h>

enum scan_matching_method {MBICP,PSM,PLICP};

/**
\brief Nonholonomic constant velocity motion model

Extended Kalman Filter using a nonholonomic constant velocity
motion model, by other works a simplistic non-linear car model.
*/
class constant_velocity_nh: public Kalman::EKFilter<double,1,false,false,true>
{
	public:
		/**
		\brief Constructor
		*/
		constant_velocity_nh(double _l,double _dt,scan_matching_method _method);
		
		///Wheel base of the nonholonomic vehicle
		double l;
		///Iteration interval
		double dt;

		/**
		\brief Update the iteration interval
		*/
		void updateDt(double _dt);
			
	protected:
		
		/**
		\brief Make the process Jacobian matrix
		*/
		void makeA();
		
		/**
		\brief Make measurement sensitivity matrix
		*/
		void makeH();
		
		/**
		\brief Make process noise sensitivity matrix
		*/
		void makeW();
	
		/**
		\brief Make measurement noise sensitivity matrix
		*/
		void makeV();

		/**
		\brief Make measurement noise covariance matrix
		*/
		void makeR();

		/**
		\brief Make process noise covariance matrix
		*/
		void makeQ();

		/**
		\brief Make process, model iteration
		*/
		void makeProcess();
		
		/**
		\brief Make measurement, used when measurement is not possible (i'm not using it now)
		*/
		void makeMeasure();
		
		///Scan matching method used, this will influence the errors
		scan_matching_method method;
};

typedef constant_velocity_nh::Vector Vector;
typedef constant_velocity_nh::Matrix Matrix;

#endif
