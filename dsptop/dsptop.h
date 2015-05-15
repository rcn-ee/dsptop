/*
 * dsptop.h
 *
 * dsptop Implementation
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef DSPTOP_H
#define DSPTOP_H

#include <stdio.h>
#include <stdint.h>

typedef enum {
    MOVING_AVERAGE_MODE, 
    TOTAL_MODE,
    LOGGING_MODE
} dsptop_mode_t;  

/* State definitions
 * Uninitialized - ETB trace has not been configured.
 * Waiting - Initialized but recording has never started.
 * Stopped - Was recording, but now stopped.
 * Recording - Obviously the receiver is recording.
 */
typedef enum {
    STATE_UNINITIALIZED,
    STATE_WAITING,
    STATE_STOPPED,
    STATE_RECORDING
} recording_state_t;

typedef enum {
    ETB_MODE_ONESHOT_FIXED,
    ETB_MODE_ONESHOT_CIRCULAR,
    ETB_MODE_DRAIN,
    ETB_MODE_LAST
} etb_mode_t;

typedef struct {
    bool resize_window_event;
    bool etb_active;
    etb_mode_t etb_mode;
    recording_state_t recording;
} global_state_t;

typedef struct {
    char * name;
    int num_dsps;
    bool k2_srss;               /* Device has Keystone 2 SmartReflex Sub System */
    int num_masters;
    uint8_t * master_id;
    char ** master_names;
} dsp_type_t;

/* Test Modes */
typedef enum {
    TEST_MODE_DISABLED,
    TEST_MODE_SMALL_DATASET,     /* Single shot - one example of each message */
    TEST_MODE_LARGE_DATASET,     /* Single shot - 5K idle and run messages */
    TEST_MODE_TIME_FAST,         /* Each message type is sent (x8) on each timer signal */
    TEST_MODE_TIME_SLOW,         /* Each message type is sent (x8) on each timer signal - 1 message type per second */
} test_mode_t;  

extern global_state_t global_state;

extern const int g_major_version;
extern const int g_minor_version;
extern const int g_patch_version;
extern const int g_copyright_year;

extern FILE *g_stdout;
extern FILE *g_stderr;
extern char * g_whoami;
extern test_mode_t g_mva_test_mode;
extern int g_total_ulm_msgcnt;

void term();
#endif /* DSPTOP_H */
