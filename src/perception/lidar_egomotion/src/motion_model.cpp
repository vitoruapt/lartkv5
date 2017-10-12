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
\brief Nonholonomic motion model source code
*/

#include "motion_model.h"

constant_velocity_nh::constant_velocity_nh(double _l,double _dt,scan_matching_method _method) 
{
	setDim(6, 0, 6, 2, 2);

	l=_l;
	dt=_dt;
	method=_method;
	
	static Vector x(6);
	double _P0[36];
	
	memset(_P0,0,sizeof(_P0));
	
	_P0[0]=100*100;
	_P0[7]=100*100;
	_P0[14]=100*100;
	_P0[21]=100*100;
	_P0[28]=100*100;
	_P0[35]=100*100;
	
	static Matrix P0(6,6,_P0);
	
	x(1)=0.;
	x(2)=0;
	x(3)=0;
	x(4)=0;
	x(5)=0;
	x(6)=0;
	
	init(x,P0);
}

void constant_velocity_nh::updateDt(double _dt)
{
	dt=_dt;
}

void constant_velocity_nh::makeA()
{
	double v1=x(3);
	double t=x(4);
	double f=x(5);
	
// 	printf("Make A dt %f\n",dt);
	
	//dF1
	A(1,1)=1.0;
	A(1,2)=0.0;
	A(1,3)=cos(t)*cos(f)*dt;
	A(1,4)=-sin(t)*cos(f)*v1*dt;
	A(1,5)=cos(t)*(-sin(f))*v1*dt;
	A(1,6)=0.0;
	
	A(2,1)=0.0;
	A(2,2)=1.0;
	A(2,3)=sin(t)*cos(f)*dt;
	A(2,4)=cos(t)*cos(f)*v1*dt;
	A(2,5)=sin(t)*(-sin(f))*v1*dt;
	A(2,6)=0.0;
	
	A(3,1)=0.0;
	A(3,2)=0.0;
	A(3,3)=1.0;
	A(3,4)=0.0;
	A(3,5)=0.0;
	A(3,6)=0.0;
	
	A(4,1)=0.0;
	A(4,2)=0.0;
	A(4,3)=sin(f/l)*dt;
	A(4,4)=1.0;
	A(4,5)=cos(f/l)/l*v1*dt;
	A(4,6)=0.0;
	
	A(5,1)=0.0;
	A(5,2)=0.0;
	A(5,3)=0.0;
	A(5,4)=0.0;
	A(5,5)=1.0;
	A(5,6)=dt;
	
	A(6,1)=0.0;
	A(6,2)=0.0;
	A(6,3)=0.0;
	A(6,4)=0.0;
	A(6,5)=0.0;
	A(6,6)=1.0;
}

void constant_velocity_nh::makeH()
{
	H(1,1) = 0.0;
	H(1,2) = 0.0;
	H(1,3) = 1.0;
	H(1,4) = 0.0;
	H(1,5) = 0.0;
	H(1,6) = 0.0;

	H(2,1) = 0.0;
	H(2,2) = 0.0;
	H(2,3) = 0.0;
	H(2,4) = 0.0;
	H(2,5) = 1.0;
	H(2,6) = 0.0;
}

void constant_velocity_nh::makeW()
{
	W(1,1) = 1.0;
	W(1,2) = 0.0;
	W(1,3) = 0.0;
	W(1,4) = 0.0;
	W(1,5) = 0.0;
	W(1,6) = 0.0;
	
	W(2,1) = 0.0;
	W(2,2) = 1.0;
	W(2,3) = 0.0;
	W(2,4) = 0.0;
	W(2,5) = 0.0;
	W(2,6) = 0.0;
	
	W(3,1) = 0.0;
	W(3,2) = 0.0;
	W(3,3) = 1.0;
	W(3,4) = 0.0;
	W(3,5) = 0.0;
	W(3,6) = 0.0;
	
	W(4,1) = 0.0;
	W(4,2) = 0.0;
	W(4,3) = 0.0;
	W(4,4) = 1.0;
	W(4,5) = 0.0;
	W(4,6) = 0.0;
	
	W(5,1) = 0.0;
	W(5,2) = 0.0;
	W(5,3) = 0.0;
	W(5,4) = 0.0;
	W(5,5) = 1.0;
	W(5,6) = 0.0;
	
	W(6,1) = 0.0;
	W(6,2) = 0.0;
	W(6,3) = 0.0;
	W(6,4) = 0.0;
	W(6,5) = 0.0;
	W(6,6) = 1.0;
}

void constant_velocity_nh::makeProcess()
{
// 	x(1) -> x
// 	x(2) -> y
// 	x(3) -> v1
// 	x(4) -> t
// 	x(5) -> f
// 	x(6) -> v2
	
	Vector x_(x.size());
	
	x_(1) = x(1) + cos(x(4))*cos(x(5))*x(3)*dt;
	x_(2) = x(2) + sin(x(4))*cos(x(5))*x(3)*dt;
	x_(3) = x(3);
	x_(4) = x(4) + sin(x(5)/l)*x(3)*dt;
// 	printf("L %f Dt %f V1 %f\n",l,dt,x(3));
// 	printf("X(4) %f\n",x_(4));
	
	x_(5) = x(5) + x(6)*dt*0.1;
	x_(6) = x(6);
	
	x.swap(x_);
}

void constant_velocity_nh::makeMeasure()
{
	z(1)=x(3);
	z(2)=x(5);
}

void constant_velocity_nh::makeV()
{
	V(1,1) = 1.0;
	V(1,2) = 0.0;
	V(2,1) = 0.0;
	V(2,2) = 1.0;
}

void constant_velocity_nh::makeR()
{
// 	double r=100*100;
// 	double r=10*10;
	
// 	double vmin=0.1;
// 	double vmax=5.;
// 	
// 	double f=(-1/vmax)*x(3)+1;
// 	f=f>0?f:0;
// 	
// 	printf(">>F %f\n",f);
// 	double f;
// 	
// 	f=10./grxy;
// 	
// 	if(isnan(f))
// 		f=100;
// 
// 	if(f>100)
// 		f=100;
// 	
// 	if(f<1)
// 		f=1;
	
// 	printf("F %f\n",f);
	/*
	if(grxy<4)
	{
		f=100;
	}else if(grxy<10)
	{
		f=1;
	}else
		f=1;*/
	
// 	0.0013
// 	0.0680
	
	
// 	R(1,1) = 0.0680;//vel
// 	R(1,2) = 0;
	
// 	R(2,1) = 0;//dir
// 	R(2,2) = 0.0013;
	
	switch(method)
	{
		case MBICP:
// 			R(1,1) = 0.2221;//vel
// 			R(1,2) = -0.0056;
			
// 			R(2,1) = -0.0056;//dir
// 			R(2,2) = 0.0036;
			
			R(1,1) = 0.0600;//vel
			R(1,2) = 0.0010;
			
			R(2,1) = 0.0010;//dir
			R(2,2) = 0.0400;
			 
			break;
			
		case PSM:
			
			R(1,1) = 0.0600;//vel
			R(1,2) = 0.0010;
			
			R(2,1) = 0.0010;//dir
			R(2,2) = 0.0400;
			
			break;
			
		case PLICP:
			R(1,1) = 0.0662;//vel
			R(1,2) = 0.0010;
			
			R(2,1) = 0.0010;//dir
			R(2,2) = 0.0217;
			break;
	}


// 	R(1,1) = r*5.;//vel
// 	R(1,2) = 0;
	
// 	R(2,1) = 0;//dir
// 	R(2,2) = r;
}

void constant_velocity_nh::makeQ()
{
	double q=0;
// 	double q=2*2;
	switch(method)
	{
		case MBICP:
// 			q=0.015;
			q=0.0015;
			break;
			
		case PSM:
			q=0.0015;
			break;
			
		case PLICP:
			q=0.0005;
			break;
	}
	
	
	
	//X
	Q(1,1) = q;
	Q(1,2) = 0.0;
	Q(1,3) = 0.0;
	Q(1,4) = 0.0;
	Q(1,5) = 0.0;
	Q(1,6) = 0.0;
	
	//Y
	Q(2,1) = 0.0;
	Q(2,2) = q;
	Q(2,3) = 0.0;
	Q(2,4) = 0.0;
	Q(2,5) = 0.0;
	Q(2,6) = 0.0;
	
	//Vl
	Q(3,1) = 0.0;
	Q(3,2) = 0.0;
	Q(3,3) = q*1.;
	Q(3,4) = 0.0;
	Q(3,5) = 0.0;
	Q(3,6) = 0.0;
	
	//Theta
	Q(4,1) = 0.0;
	Q(4,2) = 0.0;
	Q(4,3) = 0.0;
	Q(4,4) = q;
	Q(4,5) = 0.0;
	Q(4,6) = 0.0;
	
	//Fi
	Q(5,1) = 0.0;
	Q(5,2) = 0.0;
	Q(5,3) = 0.0;
	Q(5,4) = 0.0;
	Q(5,5) = q*1.;
	Q(5,6) = 0.0;
	
	//Df
	Q(6,1) = 0.0;
	Q(6,2) = 0.0;
	Q(6,3) = 0.0;
	Q(6,4) = 0.0;
	Q(6,5) = 0.0;
	Q(6,6) = q*1.;
	
}
