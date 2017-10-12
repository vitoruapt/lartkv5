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
#ifndef _DEFINE_TRAJECTORIES_H_
#define _DEFINE_TRAJECTORIES_H_

/**
 * @brief Trajectories definitions
 * @details 'a' is the ackermenn angle in degrees, and 'arc' is the distance travelled with the correspondent ackermann angle (in meters)
 */

#ifdef _trajectory_planner_nodelet_CPP_

double a[20][9] = 
// double a[3][8] = 
{
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-20, 21, 21, 21, -20,-21,-21,-21,-21},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-15, 19, 19, 19, -19,-19,-19,-19,-19},
{-20, 19.6, 19.6, 19.6, 19.6, -19.6, -19.6, -19.6, -19.6},
{-20, 19.6, 19.6, 19.6, 19.6, -19.6, -19.6, -19.6, -19.6},
{18,18,18,18,18,-20,-20,-20,-20},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},

{-17, 21, 17, 17, -21,-21,-21,-21,-21},
{-15, 21, 21, 21, -20,-21,-21,-21,-21},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-15, 19, 19, 19, -19,-19,-19,-19,-19},
{-20, 19.6, 19.6, 19.6, 19.6, -19.6, -19.6, -19.6, -19.6},
{-20, 15, 15, 22, 22, -22, -22, -22, -22},
{20,20,18,18,18,-20,-20,-20,-20},
{-15, 21, 21, 21, -21,-21,-21,-21,-21},
{-15, 21, 21, 21, -21,-21,-21,-21,-21}
};

double arc[20][9] = 
// double arc[3][8] = 
{
{0.25,-0.25,-0.3,-0.3,-0.35,-0.35,-0.25,-0.30,-0.25},
{0.5,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25},
{0.60,-0.250,-0.250,-0.250,-0.250,-0.250,-0.300,-0.350,-0.350},
{0.60,-0.250,-0.250,-0.250,-0.250,-0.250,-0.350,-0.350,-0.350},
{0.60,-0.250,-0.250,-0.250,-0.250,-0.250,-0.350,-0.250,-0.250},
{0.3,-0.20,-0.20,-0.20,-0.20,-0.20,-0.20,-0.20,-0.20},
{0.3,-0.25,-0.20,-0.20,-0.20,-0.20,-0.20,-0.20,-0.20},
{-0.250,-0.250,-0.250,-0.250,-0.250,-0.250,-0.250,-0.250,-0.25},
{0.60,-0.270,-0.270,-0.270,-0.250,-0.250,-0.250,-0.250,-0.25},
{0.60,-0.30,-0.30,-0.30,-0.250,-0.250,-0.250,-0.250,-0.25},

{0.25,-0.50,-0.25,-0.25,-0.25,-0.45,-0.30,-0.30,-0.25},
{0.5,-0.25,-0.25,-0.25,-0.35,-0.35,-0.35,-0.25,-0.15},
{0.60,-0.450,-0.250,-0.250,-0.250,-0.250,-0.350,-0.350,-0.350},
{0.60,-0.250,-0.250,-0.350,-0.250,-0.250,-0.350,-0.350,-0.350},
{0.60,-0.250,-0.250,-0.250,-0.250,-0.250,-0.350,-0.250,-0.250},
{0.3,-0.30,-0.30,-0.20,-0.20,-0.50,-0.30,-0.20,-0.20},
{0.40,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25},
{-0.250,-0.250,-0.250,-0.250,-0.250,-0.250,-0.250,-0.250,-0.25},
{0.60,-0.270,-0.270,-0.270,-0.250,-0.250,-0.250,-0.250,-0.25},
{0.60,-0.30,-0.30,-0.30,-0.250,-0.250,-0.250,-0.250,-0.25}

};
#else

extern double a[20][9];
extern double arc[20][9];
#endif
#endif

