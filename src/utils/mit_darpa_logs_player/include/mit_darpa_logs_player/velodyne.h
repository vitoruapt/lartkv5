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
 * @brief header file with velodyne types
 *@{
 */
#ifndef _VELODYNE_H
#define _VELODYNE_H

#include <stdint.h>
#include <math.h>

/** Lasers 0-31: lower lasers
          32-63: upper lasers 
**/

#define VELODYNE_NUM_LASERS 64

typedef struct velodyne_decoder velodyne_decoder_t;
struct velodyne_decoder
{
    uint8_t *data;
    uint8_t *p;
    int      data_remaining;

    int      i; // which laser within the current block are we processing?
    int      laser_offset;
    double   ctheta, sin_ctheta, cos_ctheta;

    int32_t revolution_count;
    char    version_string[16];
};

typedef struct velodyne_sample velodyne_sample_t;
struct velodyne_sample
{
    double  xyz[3];            // calibrated, projected into velodyne coordinate system
    double  raw_range;         // raw return directly from sensor
    double  range;             // corrected range
    double  ctheta;            // theta of sensor head at time of sample, **always [0, 2*PI)**
    double  theta;             // calibrated theta (horizontal/yaw angle) **always [0, 2*PI)**
    double  phi;               // calibrated phi (veritcle/pitch angle)
    double  intensity;         // normalized intensity [0, 1]
    int     physical;          // physical laser number (0-31 lower, 32-63 upper)
    int     logical;           // logical laser number (in order of increasing pitch)
};

struct velodyne_laser_calib
{
    double rcf;                // radians (rotational/yaw offset)
    double vcf;                // radians (vertical offset)
    double hcf;                // meters (horizontal off-axis offset)
    double range_offset;       // meters
    double range_scale_offset; // (scalar)
};

typedef struct 
{
    struct velodyne_laser_calib lasers[VELODYNE_NUM_LASERS]; // physical idx
    int physical2logical[VELODYNE_NUM_LASERS];
    int logical2physical[VELODYNE_NUM_LASERS];
    double sincos[VELODYNE_NUM_LASERS][2];
} velodyne_calib_t;

void velodyne_dump_calib(velodyne_calib_t *v);


// Velodyne lidars are numbered in two different ways:
// Physical: The order given by the hardware.
// Logical: In order of increasing phi

velodyne_calib_t *velodyne_calib_create();

// NOT REENTRANT. 
// Compute the logical laser numbers given the current phi angles
// (necessary before calls to p2l or l2p)
int velodyne_calib_precompute(velodyne_calib_t *v);

static inline int velodyne_physical_to_logical(velodyne_calib_t *v, int phys)
{
    return v->physical2logical[phys];
}

static inline int velodyne_logical_to_physical(velodyne_calib_t *v, int logical)
{
    return v->logical2physical[logical];
}

// return an upper bound on the # of samples in this message
int velodyne_decoder_estimate_samples(velodyne_calib_t *v, const void *_data, int datalen);

int velodyne_decoder_init(velodyne_calib_t *v, velodyne_decoder_t *vd, const void *_data, int datalen);
int velodyne_decoder_next(velodyne_calib_t *v, velodyne_decoder_t *vd, struct velodyne_sample *sample);

#endif
/**
 *@}
 */      
