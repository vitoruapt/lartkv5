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
 * @brief Utilities for velodyne laser
 *@{
 */


#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "math_util.h"

#include "velodyne.h"

#define UPPER_MAGIC 0xeeff
#define LOWER_MAGIC 0xddff

#include "velodyne-newunit.h"

#define RADIANS_PER_LSB 0.00017453293
#define METERS_PER_LSB 0.002

// return an upper bound on the # of samples in this message
int velodyne_decoder_estimate_samples(velodyne_calib_t *v, const void *_data, 
        int datalen)
{
    return (datalen / 3) +1;
}

int velodyne_decoder_init(velodyne_calib_t *v, velodyne_decoder_t *vd, 
        const void *_data, int datalen)
{
    vd->data    = (uint8_t*) _data;
    vd->data_remaining = datalen;
    vd->i       = 0;

    if (datalen != 1206) {
        printf("velodyne: bad len %i\n", datalen);
        vd->data_remaining = 0; // don't decode it.
        return -1;
    }

    // copy out the version string
    if (vd->data[1202]=='v') {
      vd->revolution_count = vd->data[1200] + (vd->data[1201]<<8);
      memcpy(vd->version_string, &vd->data[1202], 4);
      vd->version_string[4]=0;
    } else {
      vd->version_string[0]=0;
      vd->revolution_count=-1;
    }

    return 0;
}

int velodyne_decoder_next(velodyne_calib_t *v, velodyne_decoder_t *vd, 
        velodyne_sample_t *sample)
{
    // if we finished the last block, "consume" this block of data.
    if (vd->i == 32) {
        vd->data_remaining -= 100;
        vd->data += 100;
        vd->i = 0;
    }

    uint8_t *data = vd->data;

    // starting a new block? 
    if (vd->i == 0) {
        // enough data for another block?
        if (vd->data_remaining < 100)
            return -1;

        int magic = data[0] + (data[1]<<8);

        if (magic == UPPER_MAGIC) 
            vd->laser_offset = 32;
        else if (magic == LOWER_MAGIC)
            vd->laser_offset = 0;
        else {
            printf("Unknown velodyne magic %4x\n", magic);
            return -2;
        }
      
        // position of velodyne head, constant for all 32 measurements that
        // follow
        vd->ctheta = 2*PI - (data[2] + (data[3]<<8)) * RADIANS_PER_LSB;
        if (vd->ctheta == 2*PI)
            vd->ctheta = 0;
        
        vd->sin_ctheta = sin(vd->ctheta);
        vd->cos_ctheta = cos(vd->ctheta);
        vd->i = 0;
    }

    // Decode the laser sample
    int      i       = vd->i;
    
    sample->physical = vd->laser_offset + i;
    sample->logical  = velodyne_physical_to_logical(v, sample->physical);

    struct velodyne_laser_calib *params = &v->lasers[sample->physical];

    sample->raw_range = (data[4 + i*3] + (data[5+i*3]<<8)) * METERS_PER_LSB;
    sample->range     = (sample->raw_range + params->range_offset) * (1.0 + params->range_scale_offset);
    sample->ctheta    = vd->ctheta;
    sample->theta     = mod2pi_ref(PI, vd->ctheta + params->rcf);
    sample->phi       = params->vcf;
    sample->intensity = data[6 + i*3]/255.0;

    double sin_theta, cos_theta;
    sin_theta = sin(sample->theta);
    cos_theta = cos(sample->theta);
    double sin_phi = v->sincos[sample->physical][0];
    double cos_phi = v->sincos[sample->physical][1];

    sample->xyz[0] = sample->range * cos_theta * cos_phi;
    sample->xyz[1] = sample->range * sin_theta * cos_phi;
    sample->xyz[2] = sample->range * sin_phi;
  
    // handle horizontal offset ("parallax")
    sample->xyz[0] -= params->hcf * vd->cos_ctheta;
    sample->xyz[1] -= params->hcf * vd->sin_ctheta;
    vd->i++;

    // successful decode
    return 0;
}

int velodyne_decode(velodyne_calib_t *v, const void *_data, int datalen, 
                    double *_theta0,
                    double *ranges, double *intensities, double *thetas, 
                    double *phis, int *laserids, int *nsamples, int *badscans)
{
    if (datalen != 1206) {
        printf("velodyne: bad len %i\n", datalen);
        return -1;
    }

    int out_idx = 0;
    int laser_offset = 0;

    uint8_t *data;
    for (data = (uint8_t*) _data; 
            datalen >= 100; 
            datalen-=100, data += 100) {

        int magic = data[0] + (data[1]<<8);

        if (magic == UPPER_MAGIC) 
            laser_offset = 32;
        else if (magic == LOWER_MAGIC)
            laser_offset = 0;
        else {
            printf("Unknown velodyne magic %4x\n", magic);
            continue;
        }

        double theta0 = 2*PI - (data[2] + (data[3]<<8)) * RADIANS_PER_LSB;
	*_theta0 = theta0;

        int i;
        for (i = 0; i < 32; i++) {
            struct velodyne_laser_calib *params = &v->lasers[laser_offset + i];
            
            ranges[out_idx] = (data[4 + i*3] + (data[5+i*3]<<8)) * METERS_PER_LSB;
            ranges[out_idx] *= (1.0 + params->range_scale_offset);
            ranges[out_idx] -= params->range_offset;
            
            // skip illegally short ranges
            if (ranges[out_idx] < 0.5) {
                badscans[laser_offset+i]++;
                continue;
            }

            intensities[out_idx] = data[6 + i*3]/255.0;
            thetas[out_idx] = theta0 + params->rcf;
            phis[out_idx] = params->vcf;
            laserids[out_idx] = laser_offset + i;
            out_idx++;
        }
    }

    *nsamples = out_idx;
    // last six bytes are status. we don't know what to do with them.

    return 0;
}

void velodyne_calib_dump(velodyne_calib_t *v)
{
    printf("struct velodyne_laser_calib velodyne_NAME_ME_HERE[] = {\n");
    int i;
    for (i = 0; i < VELODYNE_NUM_LASERS; i++) {
        struct velodyne_laser_calib *params = &v->lasers[i];
        printf("   { %11.7f, %11.7f, %8.4f, %8.4f, %10.6f }, // laser %2d\n", 
	       params->rcf, params->vcf, params->hcf, params->range_offset, params->range_scale_offset, i);
    }
    printf("};\n\n");
}


static velodyne_calib_t *__v;
static int laser_phi_compare(const void *_a, const void *_b)
{
    int a = *((int*) _a);
    int b = *((int*) _b);

    if (__v->lasers[a].vcf < __v->lasers[b].vcf) 
        return -1;
    return 1;
}

// NOT REENTRANT
int velodyne_calib_precompute(velodyne_calib_t *v)
{
    assert (!__v); // check for reentrancy...

    __v = v;

    int i;
    for (i = 0; i < VELODYNE_NUM_LASERS; i++)
        v->logical2physical[i] = i;
    qsort(v->logical2physical, VELODYNE_NUM_LASERS, sizeof(int), 
            laser_phi_compare);
    
    int logical;
    for (logical = 0; logical < VELODYNE_NUM_LASERS; logical++) {
        v->physical2logical[v->logical2physical[logical]] = logical;
    }

    int physical;
    for (physical = 0; physical < VELODYNE_NUM_LASERS; physical++) {
        v->sincos[physical][0] = sin(v->lasers[physical].vcf);
        v->sincos[physical][1] = cos(v->lasers[physical].vcf);
    }
    __v = NULL;

    return 0;
}

velodyne_calib_t *velodyne_calib_create()
{
    velodyne_calib_t *v = (velodyne_calib_t*) calloc(1, sizeof(velodyne_calib_t));
    memcpy(v->lasers, velodyne_uncalibrated, sizeof(struct velodyne_laser_calib) * VELODYNE_NUM_LASERS);
    velodyne_calib_precompute(v);

    return v;
}
