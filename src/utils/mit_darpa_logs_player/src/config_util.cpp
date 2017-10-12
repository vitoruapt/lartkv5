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
 * @file config_util.cpp
 * @brief configuration utilities, such as calibrations etc.
 *@{
 */
#include <math.h>
#include <assert.h>
#include "config_util.h"
#include "math_util.h"
#include "rotations.h"
#include "small_linalg.h"

static int 
_config_util_get_quat (Config *cfg, const char *name, double quat[4])
{
    char key[256];
    sprintf(key, "calibration.%s.orientation", name);
    if (config_has_key(cfg, key)) {
        int sz = config_get_double_array(cfg, key, quat, 4);
        assert(sz==4);
        return 0;
    }

    sprintf(key, "calibration.%s.rpy", name);
    if (config_has_key(cfg, key)) {
        double rpy[3];
        int sz = config_get_double_array(cfg, key, rpy, 3);
        assert(sz == 3);
        int i;
        for (i = 0; i < 3; i++)
            rpy[i] = to_radians(rpy[i]);
        rot_roll_pitch_yaw_to_quat(rpy, quat);
        return 0;
    }

    sprintf(key, "calibration.%s.angleaxis", name);
    if (config_has_key(cfg, key)) {
        double aa[4];
        int sz = config_get_double_array(cfg, key, aa, 4);
        assert(sz==4);

        double theta = aa[0];
        double s = sin(theta/2);

        quat[0] = cos(theta/2);
        int i;
        for (i = 1; i < 4; i++)
            quat[i] = aa[i] * s;
        return 0;
    }
    return -1;
}

int config_util_get_quat(Config *cfg, const char *name, double quat[4])
{
    int result = _config_util_get_quat (cfg, name, quat);
    return result;
}

int config_util_get_pos(Config *cfg, const char *name, double pos[3])
{
    char key[256];

    sprintf(key, "calibration.%s.position", name);
    if (config_has_key(cfg, key)) {
        int sz = config_get_double_array(cfg, key, pos, 3);
        assert(sz==3);
        return 0;
    } 
    return -1;
}

int config_util_get_matrix(Config *cfg, const char *name, double m[16])
{
    double quat[4];
    double pos[3];

    if (config_util_get_quat(cfg, name, quat))
        return -1;

    if (config_util_get_pos(cfg, name, pos))
        return -1;

    rot_quat_pos_to_matrix(quat, pos, m);

    return 0;
}

// compute the sensor-to-local rigid body transformation matrix for a
// specific sensor, given a vehicle pose.
int 
config_util_sensor_to_local_with_pose(Config *config, 
        const char *name, double m[16], 
        lcmtypes_pose_t *p)
{
    double body_to_local[16];

    rot_quat_pos_to_matrix(p->orientation, p->pos, body_to_local);

    double sensor_to_calibration[16];

    if (config_util_get_matrix(config, name, sensor_to_calibration))
        return -1;

    char key[128];
    sprintf(key,"calibration.%s.relative_to", name);

    char *calib_frame = config_get_str_or_fail(config, key);
    if (!strcmp(calib_frame, "body")) {

        matrix_multiply_4x4_4x4(body_to_local, sensor_to_calibration, m);

    } else {
        double calibration_to_body[16];
        
        if (config_util_get_matrix(config, calib_frame, calibration_to_body))
            return -1;

        double sensor_to_body[16];
        matrix_multiply_4x4_4x4(calibration_to_body, sensor_to_calibration, 
                                sensor_to_body);
        matrix_multiply_4x4_4x4(body_to_local, sensor_to_body, m);
    }

    return 0;
}
/**
 *@}
 */      
