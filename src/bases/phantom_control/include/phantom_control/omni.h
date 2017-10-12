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
 * @file omni.h
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Header for the OpenHaptics lib and the haptic device state structure
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <vector>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>


#define HOME_XX 0
#define HOME_YY 0
#define HOME_ZZ 0

/**
 * @struct OmniState
 * 
 * @brief Structure for holding the state variables of the haptic device
 * @param position : end factor position
 * @param rot : last 3 DOF
 * @param joints : first 3 DOF
 * @param force : feedback to device
 * @param thetas[7] : degrees of all joints  
 * @param resz Pedido da posição para o eixo \c ZZ do pórtico.
 * @param temp : temperture of the 3 motores
 * @param buttons : state of the 2 buttons
 */
struct OmniState {
    hduVector3Dd position;  //3x1 vector of position
    //hduVector3Dd velocity;  //3x1 vector of velocity
    //hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
    //hduVector3Dd inp_vel2;  
    //hduVector3Dd inp_vel3;  
    //hduVector3Dd out_vel1;  
    //hduVector3Dd out_vel2;  
    //hduVector3Dd out_vel3;
    hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity 
    //hduVector3Dd pos_hist2; 
    hduVector3Dd rot;
    hduVector3Dd joints;
    hduVector3Dd force;     //3 element double vector force[0], force[1], force[2]
    float thetas[7];
    HDdouble temp[3];  // Motors Temperture
    int buttons[2];
    //int buttons_prev[2];
    bool home;
    hduVector3Dd home_pos;
};
