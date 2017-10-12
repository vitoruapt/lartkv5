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
/** @file 
* @brief The documentation of this file is a responsibility of its current developer, Pedro Salvado.
*/

#include <iostream>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <kfilter/ekfilter.hpp>

#define DEFAULT_WHEEL_BASE 2.5

#include <signal.h>

#include <atlascar_base/AtlascarStatus.h>
#include <atlascar_base/AtlascarVelocityStatus.h>
using namespace ros;
using namespace std;
using namespace Kalman;

void IncommingDataHandler(int);
// bool ConvertEstimatedToMeasurment(double vl,double dir,double*dx,double*dy,double*dtheta,double dt,double l,double bwa);

class non_holonomic_ekfilter: public Kalman::EKFilter<double,1,false,false,true>
{
       public:
       non_holonomic_ekfilter()
       {
              setDim(5,0,5,2,2);

              Vector x_0(5);
              double _P0[25];

              memset(_P0,0,sizeof(_P0));

              _P0[0]=1*1;
              _P0[6]=1*1;
              _P0[12]=0.1*1;
              _P0[18]=100*100;
              _P0[24]=100*100;

              x_0(1)=0;
              x_0(2)=0;
              x_0(3)=0;
              x_0(4)=0;
              x_0(5)=0;


              Matrix P0(5,5,_P0);

              cout<<"P0 "<<P0<<endl;
              cout<<"x_0 "<<x_0<<endl;

              init(x_0,P0);

  
       }

       
       void SetWheelBase(double l_)
       {
              l=l_;
       
       }

       void SetTimeInterval(double dt_)
       {
              dt=dt_;
       }

       

       protected:
       
       double l,dt;
       void makeA()
       {
              double vl=x(4);
              double yaw=x(3);
              double phi=x(5);

              A(1,1)=1.0;
              A(1,2)=0.0;
              A(1,3)=-sin(yaw)*cos(phi)*vl*dt;
              A(1,4)=cos(yaw)*cos(phi)*dt;
              A(1,5)=cos(yaw)*(-sin(phi))*vl*dt;
              
              
              A(2,1)=0.0;
              A(2,2)=1.0;
              A(2,3)=cos(yaw)*cos(phi)*vl*dt;
              A(2,4)=sin(yaw)*cos(phi)*dt;
              A(2,5)=sin(yaw)*(-sin(phi))*vl*dt;
              
              A(3,1)=0.0;
              A(3,2)=0.0;
              A(3,3)=1.0;
              //A(3,4)=sin(phi/l)*dt; //erro no l
              A(3,4)=sin(phi)*(dt/l);
              //A(3,5)=cos(phi/l)*vl*dt/l; //erro no l
              A(3,5)=cos(phi)*vl*dt/l;
         
              A(4,1)=0.0;
              A(4,2)=0.0;
              A(4,3)=0.0;
              A(4,4)=1.0;
              A(4,5)=0.0;
              
              A(5,1)=0.0;
              A(5,2)=0.0;
              A(5,3)=0.0;
              A(5,4)=0.0;
              A(5,5)=1.0;
              

              
       }
       void makeH()
       {
              
              H(1,1) = 0.0;
              H(1,2) = 0.0;
              H(1,3) = 0.0;
              H(1,4) = 1.0;
              H(1,5) = 0.0;

              H(2,1) = 0.0;
              H(2,2) = 0.0;
              H(2,3) = 0.0;
              H(2,4) = 0.0;
              H(2,5) = 1.0;
              


       }
       void makeW()
       {
              W(1,1) = 1.0;
              W(1,2) = 0.0;
              W(1,3) = 0.0;
              W(1,4) = 0.0;
              W(1,5) = 0.0;

              W(2,1) = 0.0;
              W(2,2) = 1.0;
              W(2,3) = 0.0;
              W(2,4) = 0.0;
              W(2,5) = 0.0;

              W(3,1) = 0.0;
              W(3,2) = 0.0;
              W(3,3) = 1.0;
              W(3,4) = 0.0;
              W(3,5) = 0.0;

              W(4,1) = 0.0;
              W(4,2) = 0.0;
              W(4,3) = 0.0;
              W(4,4) = 1.0;
              W(4,5) = 0.0;


              W(5,1) = 0.0;
              W(5,2) = 0.0;
              W(5,3) = 0.0;
              W(5,4) = 0.0;
              W(5,5) = 1.0;

       }
       void makeV()
       {
              V(1,1) = 1.0;
              V(1,2) = 0.0;
              V(2,1) = 0.0;
              V(2,2) = 1.0;
       }

       void makeR()
       {
//               R(1,1) = 0.1;//vel
              R(1,1) =0.00121;
        
              R(1,2) = 0.0;

              R(2,1) = 0.0;
             R(2,2) = 0.003106;//dir
//               R(2,2) = 0.01;

       }

       void makeQ()
       {
              //X   
//               double q=0.001;
              Q(1,1) = 0.000001;
              Q(1,2) = 0.0;
              Q(1,3) = 0.0;
              Q(1,4) = 0.0;
              Q(1,5) = 0.0;

              
              //Y
              Q(2,1) = 0.0;
              Q(2,2) = 0.000001;
              Q(2,3) = 0.0;
              Q(2,4) = 0.0;
              Q(2,5) = 0.0;

              
              //yaw
              Q(3,1) = 0.0;
              Q(3,2) = 0.0;
              Q(3,3) = 0.005;
              Q(3,4) = 0.0;
              Q(3,5) = 0.0;

              
              //vl
              Q(4,1) = 0.0;
              Q(4,2) = 0.0;
              Q(4,3) = 0.0;
              Q(4,4) = 0.01;
              Q(4,5) = 0.0;

              
              //phi
              Q(5,1) = 0.0;
              Q(5,2) = 0.0;
              Q(5,3) = 0.0;
              Q(5,4) = 0.0;
              Q(5,5) = 0.0005;
              

       }

       void makeProcess()
       {
              Vector x_(x.size());
              x_(1) = x(1) + cos(x(3))*cos(x(5))*x(4)*dt;
              x_(2) = x(2) + sin(x(3))*cos(x(5))*x(4)*dt;
              //x_(3) = x(3) + sin(x(5)/l)*x(4)*dt;  //possivel erro no l
              x_(3) = x(3) + sin(x(5))*(x(4)/l)*dt;  //possivel erro no l
              x_(4) = x(4);
              x_(5) = x(5);
              x.swap(x_);   
       }

       void makeMeasure()
       {
              z(1)=x(4); //velocidade
              z(2)=x(5); // phi - angulo das rodas
       }
};


typedef non_holonomic_ekfilter::Vector Vector;
typedef non_holonomic_ekfilter::Matrix Matrix;
