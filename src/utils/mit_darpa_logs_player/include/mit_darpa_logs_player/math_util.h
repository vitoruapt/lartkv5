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
 * @addtogroup mit_logs
 * @file 
 * @brief header file for math utilities
 * *@{
 */
#ifndef _MATHUTIL_H
#define _MATHUTIL_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#ifndef PI
#define PI 3.14159265358979323846264338
#endif

#define to_radians(x) ( (x) * (PI / 180.0 ))
#define to_degrees(x) ( (x) * (180.0 / M_PI ))

static inline double sq(double v)
{
  return v*v;
}

static inline double sgn(double v)
{
  return (v>=0) ? 1 : -1;
}

// random number between [0, 1)
static inline float randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}

static inline float signed_randf()
{
    return randf()*2 - 1;
}

// return a random integer between [0, bound)
static inline int irand(int bound)
{
    int v = (int) (randf()*bound);
    assert(v >= 0);
    assert(v < bound);
    return v;
}

#define TWOPI_INV (0.5/PI)
#define TWOPI (2*PI)

/** valid only for v > 0 **/
static inline double mod2pi_positive(double vin)
{
    double q = vin * TWOPI_INV + 0.5;
    int qi = (int) q;

    return vin - qi*TWOPI;
}

/** Map v to [-PI, PI] **/
static inline double mod2pi(double vin)
{
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

/** Return vin such that it is within PI degrees of ref **/
static inline double mod2pi_ref(double ref, double vin)
{
    return ref + mod2pi(vin - ref);
}

static inline int theta_to_int(double theta, int max)
{
    theta = mod2pi_ref(PI, theta);
    int v = (int) (theta / ( 2 * PI ) * max);

    if (v==max)
        v = 0;

    assert (v >= 0 && v < max);

    return v;
}

static inline int imin(int a, int b)
{
    return (a < b) ? a : b;
}

static inline int imax(int a, int b)
{
    return (a > b) ? a : b;
}

static inline int64_t imin64(int64_t a, int64_t b)
{
    return (a < b) ? a : b;
}

static inline int64_t imax64(int64_t a, int64_t b)
{
    return (a > b) ? a : b;
}

static inline int iclamp(int v, int minv, int maxv)
{
    return imax(minv, imin(v, maxv));
}

static inline double fclamp(double v, double minv, double maxv)
{
    return fmax(minv, fmin(v, maxv));
}

#endif
/**
 *@}
 */      
