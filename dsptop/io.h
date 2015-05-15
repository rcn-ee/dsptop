/*
 * io.c
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

#ifndef IO_H
#define IO_H

#include <time.h>
#include "command.h"
#include "dsptop.h"

typedef enum {
    DSPTOP_INVALID,
    DSPTOP_INVALID_LOGGING,
    DSPTOP_HELP,
    DSPTOP_DISPLAY_UPDATE,
    DSPTOP_MOVING_AVERAGE,
    DSPTOP_PLOT_RESOLUTION_UPDATE,
    DSPTOP_WRITE_CONFIG,
    DSPTOP_CLEAR,
    DSPTOP_QUIT,
    DSPTOP_NO_COMMAND
} interactive_cmd_t;

typedef struct {
    int num_1kb_blocks; 
    double percent_free;
    double min_percent_free; /* kept for testing purposes only */
} usage_mem_t;

typedef struct {
    struct timeval * run_tv_p;
    struct timeval * idle_tv_p;
    usage_mem_t extern_cd;
    usage_mem_t extern_d;
    usage_mem_t intern_d;   
    int8_t temp;
    int8_t temp_max;
}dsp_usage_info_t;

typedef struct {
    dsp_type_t * dsp_p;
    dsptop_mode_t dsptop_mode;
    struct timeval elapsed_tv;
    struct timeval gap_tv;
}dsptop_info_t;

void io_init();
void io_term();
void io_update_screen(dsptop_info_t * dsptop_info_p, dsp_usage_info_t * dsp_usage_info_p);
void io_resize_screen();
void io_help(dsptop_info_t * dsptop_info_p, options_t * options_p);
bool io_wait();
interactive_cmd_t io_get_cmd(dsptop_info_t * dsptop_info_p);
char io_getch();
void io_put_cmdpromt(interactive_cmd_t cmd, double * value_p);
double io_get_doublevalue(double value);
const char * io_get_modestr(dsptop_mode_t dsptop_mode);
void io_invalid();
void io_restore_prompt();
bool io_queue_putch(char ch);
char io_queue_getch(bool * ch_available);

#endif /* IO_H */
