/*
 * command.h
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
#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include <stdbool.h>
#include "dsptop.h"

typedef enum {
    FMT_TEXT, 
    FMT_CSV,
    FMT_GNUPLOT,
    FMT_GNUPLOT_WXT,
    FMT_LAST
} format_type_t;

typedef enum {
    LOG_FIRST,
    LOG_LAST,
    LOG_END
} logging_type_t;

typedef struct {
    char * out_filename_p;      /* Log filename */
    char * rc_filename_p;       /* Resource filename */
    format_type_t format_type;  /* Format type */
    uint32_t max_filesize;      /* Max plot and log file size in bytes */
    bool logging_enable;        /* Logging mode enabled */
    bool sync_enable;           /* Sync with script thorugh a pipe enable */
    bool stop_full;             /* ETB default is circular mode */
    bool quiet_mode;            /* Quiet/silent mode - only log values are exported out stdout */
    bool process_until_exitmsg; /* Process idle/run messsages until an exit msg is found */
    uint32_t num_iterations;    /* Number of screen update iterations, then exit */
    double delay_rate;          /* Screen refresh rate in seconds */
    double mva_window;          /* Moving average sample window in seconds */ 
    test_mode_t test_mode;      /* Test mode */
    double plot_resolution;     /* Plot resolution */
} options_t;

void parse_args (int argc, char * const argv[], options_t * options_p);
void parse_rc(options_t * options_p);
void write_rc(options_t * options_p);
char * get_option_argument(char option, int arg_index);

/* delay may only be between .5 and 900 seconds */
#define adjust_delay(value); \
            if (value < .5) {\
                value = .5;\
            }\
            if (value > 900) {\
                value = 900;\
            }

/* mva_window may only be between .5 and 900 seconds */ 
#define adjust_mva_window(value); \
            if ((value < .5) && (value != 0)) {\
                value = .5;\
            }\
            if (value > 900) {\
                value = 900;\
            }

#define adjust_plot_resolution(value); \
            if (value < .001) {\
                value = .001;\
            }\
            if (value > 1) {\
                value = 1;\
            }

#endif /* COMMAND_H */

