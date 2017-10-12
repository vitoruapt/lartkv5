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
 * @brief header file for eventlog.cpp Check the LCM instructions for more info
 *@{
 */
#ifndef _LCM_EVENTLOG_H_
#define _LCM_EVENTLOG_H_

#include <stdio.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _lcm_eventlog_t lcm_eventlog_t;
struct _lcm_eventlog_t
{
    FILE *f;
    int64_t eventcount;
};

typedef struct _lcm_eventlog_event_t lcm_eventlog_event_t;
struct _lcm_eventlog_event_t {
    int64_t eventnum, timestamp;
    int32_t channellen, datalen;

    char     *channel;
    void     *data;
};

// mode must be "r"
lcm_eventlog_t *lcm_eventlog_create(const char *path, const char *mode);

// when you're done with the log, clean up after yourself!
void lcm_eventlog_destroy(lcm_eventlog_t *l);

// read the next event.
lcm_eventlog_event_t *lcm_eventlog_read_next_event(lcm_eventlog_t *l);

// free the structure returned by lcm_eventlog_read_next_event
void lcm_eventlog_free_event(lcm_eventlog_event_t *le);

// seek (approximately) to particular timestamp
int lcm_eventlog_seek_to_timestamp(lcm_eventlog_t *l, int64_t ts);

#ifdef __cplusplus
}
#endif

#endif
/**
 *@}
 */      
