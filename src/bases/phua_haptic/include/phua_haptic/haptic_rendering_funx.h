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
 * @brief haptic_rendering_funx.h file for this module. Contains includes, prototypes and global vars.
 * @author pedro_cruz
 * @version 1.0
 * @date 10 July 2012
 *@{
 */
#ifndef _HAPTIC_RENDERING_FUNX_H
#define _HAPTIC_RENDERING_FUNX_H

/*~~~~~~~~~~~~~~~~~ 
|| OpenHaptics 3  ||
~~~~~~~~~~~~~~~~~~*/
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HL/hl.h>

#include <phua_haptic/types.h>
#include <phua_haptic/miscellaneous.h>

#include <phua_haptic/hd_hl_apis_aux.h>

/** 
* @brief Function to define the workspace haptic demo force.
* @param pUserData pointer to shared structure.
* @return force vector.
*/
hduVector3Dd CalculateWorkspaceDemoForce(void *pUserData);

/** 
* @brief Function to define the plane drawing haptic demo force.
* @param pUserData pointer to shared structure.
* @return force vector.
*/
hduVector3Dd CalculatePlaneDrawingDemoForce(void *pUserData);

/** 
* @brief Function to define the plane for plane drawing demo.
* @param pUserData pointer to shared structure.
* @return none.
*/
void Demo2_BuildPlane(void *pUserData);

/** 
* @brief Function to determine force values using the tanh customized force behaviour law.
* @param x distance to obstacle.
* @return force value.
*/
template <typename Type>
Type CalculateTanhForceMagnitude(Type x)
{
  // constants
  static Type overlap = 30.0; //mm
  static Type limit = 0.25 * overlap; //mm
  static Type Fmax = 2.5; //N
  static Type k = Fmax / overlap; //N/mm
  static Type gamma = 0.2;
  static Type k2 = 10.0;
  
  // calcs
  Type F;
  
  if(x<overlap && x>=limit)
    F = k * gamma * x;
  else if(x>0.0 && x<limit)
  {
    Type beta = log(-(-exp(limit*3.0)+exp(limit)+Fmax*exp(limit)-exp(limit)*sqrt((exp(limit)+1.0)*(Fmax+gamma*k*limit-gamma*k*overlap)*(Fmax-exp(limit)*2.0+Fmax*exp(limit)+gamma*k*limit-gamma*k*overlap+gamma*k*limit*exp(limit)-gamma*k*overlap*exp(limit)+2.0))+Fmax*exp(limit*3.0)+exp(limit*2.0)*sqrt((exp(limit)+1.0)*(Fmax+gamma*k*limit-gamma*k*overlap)*(Fmax-exp(limit)*2.0+Fmax*exp(limit)+gamma*k*limit-gamma*k*overlap+gamma*k*limit*exp(limit)-gamma*k*overlap*exp(limit)+2.0))+gamma*k*limit*exp(limit)-gamma*k*overlap*exp(limit)+gamma*k*limit*exp(limit*3.0)-gamma*k*overlap*exp(limit*3.0))/(-exp(limit*2.0)+Fmax*exp(limit)*2.0+gamma*k*limit*exp(limit)*2.0-gamma*k*overlap*exp(limit)*2.0+1.0))*(1.0/2.0);
    
    Type alpha = pow(cosh(beta - (limit/2.0)),2.0);
    
    Type theta = Fmax + tanh(beta)/(pow(tanh(beta - (limit/2.0)),2.0)-1.0);
    
    F = alpha * tanh(beta - x) + theta;
  }
  else if(x<=0.)
    F = Fmax + k2 * -x;
  else
    F = 0.;
  
  return F;
}

/** 
* @brief Function that tracks the detached leg COP and creates a force vector.
* @param pUserData pointer to shared structure.
* @return force vector.
*/
hduVector3Dd Leg_COP_Monitoring(void *pUserData);

/** 
* @brief Function to determine force values using the polynomial force behaviour law.
* @param x distance to obstacle.
* @return force value.
*/
template <typename Type>
Type CalculatePolyForceMagnitude(Type x)
{
  // overlap zone -> define an area where haptics and control overlap
  static Type overlap_zone = 5.0;
  
  // force polynomial variables
  static Type Fmax = 3.3; // N;
  static const Type k = Fmax / overlap_zone; // N/mm
  static const Type k2 = 100.0*k; // N/mm
  static Type a = 2.0 * Fmax / pow(overlap_zone,3.0);
  static Type b = -1.0 * 3.0 * Fmax / pow(overlap_zone,2.0);
  
  Type F;
  
  if(x<=overlap_zone && x>0.0)
  {
    F = (a * pow(x,3.) + b * pow(x,2.) + Fmax);
  }
  else if(x<=0.0)
  {
    F = Fmax + k2 * (-x);
  }
  else
  {
    F = 0.0;
  }
  
  return F;
}

/** 
* @brief Function to determine force values using the exponential force behaviour law.
* @param x distance to obstacle.
* @return force value.
*/
template <typename Type>
Type CalculateExponentialForceMagnitude(Type x)
{
  return exp(-x+1.0);
}

/** 
* @brief Function to determine force values of the plane in the plane drawing demo.
* @param x distance to obstacle.
* @return force value.
*/
template <typename Type>
Type CalculatePlaneForceMagnitude(Type x)
{
  return CalculateTanhForceMagnitude(x);
}

#endif
/**
 *@}
*/
