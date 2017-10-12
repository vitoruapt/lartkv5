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
 * @brief Given a log file, extract velodyne measurements within a specified
time window.  Measurements are projected into the local frame and 
written to stdout as a point cloud.
 *@{
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "config_util.h"
#include "eventlog.h"
#include "lcmtypes_velodyne_t.h"
#include "lcmtypes_pose_t.h"
#include "small_linalg.h"
#include "velodyne.h"

int main(int argc, char **argv)
{
    if(argc < 4) {
        fprintf(stderr, 
                "usage: example4-velodyne <logfile> <start_time> <end time>\n"
                "\n"
                "start_time and end_time are given in seconds from the\n"
                "start of the log file\n");
        return 1;
    }

    lcm_eventlog_t *log = lcm_eventlog_create(argv[1], "r");
    if(!log) {
        fprintf(stderr, "error opening log file\n");
        return 1;
    }

    double start_time = strtod(argv[2], NULL);
    double end_time = strtod(argv[3], NULL);

    Config *config = config_parse_default();
    if(!config) {
        fprintf(stderr, "couldn't find config file\n");
        return 1;
    }

    // load the velodyne sensor calibration
    velodyne_calib_t *vcalib = velodyne_calib_create();

    // read the first timestamp of the log file
    lcm_eventlog_event_t *event = lcm_eventlog_read_next_event(log);
    int64_t first_timestamp = event->timestamp;

    // compute the desired start and end timestamps
    int64_t start_utime = first_timestamp + (int64_t)(start_time * 1000000);
    int64_t end_utime = first_timestamp + (int64_t)(end_time * 1000000);

    lcmtypes_pose_t last_pose;
    memset(&last_pose, 0, sizeof(last_pose));

    while(1) {
        // release the last event
        lcm_eventlog_free_event(event);

        // read an event
        event = lcm_eventlog_read_next_event(log);

        if(!event)
            break;

        // always keep track of the current pose
        if(!strcmp(event->channel, "POSE")) {
            if(last_pose.utime) 
                lcmtypes_pose_t_decode_cleanup(&last_pose);

            lcmtypes_pose_t_decode(event->data, 0, event->datalen, &last_pose);
        }

        // ignore other messages until the desired start time
        if(event->timestamp < start_utime) {
            continue;
        }

        // quit if we're done
        if(event->timestamp >= end_utime) {
            break;
        }

        if(!strcmp(event->channel, "VELODYNE")) {
            // parse the LCM packet into a velodyne data packet.
            lcmtypes_velodyne_t vel;
            lcmtypes_velodyne_t_decode(event->data, 0, event->datalen, 
                    &vel);

            // compute the velodyne-to-local transformation matrix
            // 
            // This is an approximation because we're using the last recorded
            // pose.  A more accurate projection might be to project the
            // vehicle's pose forward based on its last measured velocity.
            double velodyne_to_local[16];
            config_util_sensor_to_local_with_pose (config, "VELODYNE", 
                velodyne_to_local, &last_pose);

            // parse the velodyne data packet
            velodyne_decoder_t vdecoder;
            velodyne_decoder_init(vcalib, &vdecoder, vel.data, vel.datalen);

            // project each sample in the velodyne data packet into the local
            // frame
            velodyne_sample_t vsample;
            while (!velodyne_decoder_next(vcalib, &vdecoder, &vsample)) {
                if (vsample.range < 0.01) {
                    continue;
                }

                double sensor_xyz[4] = { 
                    vsample.xyz[0], vsample.xyz[1], vsample.xyz[2], 1 
                };
                double local_xyz[4];
                matrix_vector_multiply_4x4_4d(velodyne_to_local, 
                        sensor_xyz, local_xyz);

                printf("%f %f %f\n", local_xyz[0], local_xyz[1], local_xyz[2]);
            }

            lcmtypes_velodyne_t_decode_cleanup(&vel);
        }

    }
    lcm_eventlog_free_event(event);

    lcm_eventlog_destroy(log);

    return 0;
}
/**
 *@}
 */      
