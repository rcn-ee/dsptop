/*
 * utility.h
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
#ifndef UTILITY_H
#define UTILITY_H

#include <stdlib.h>
#include <time.h>

typedef enum { TIMESTRING_SECOND,
               TIMESTRING_MILLISECOND, 
               TIMESTRING_MICROSECOND
} timestring_resolution_t; 

void util_gen_filename(char * buf, size_t buf_size, const char * file_name, const char * file_ext);
size_t util_get_filesize(char * log_filename_p);
size_t util_get_timestring(char * time_buf_p, size_t time_buf_size, struct timeval * time_tv, timestring_resolution_t time_res);
char * util_get_device();
void util_pipe_write(const char * wr_buf_p, size_t wr_bytecnt);
void util_pipe_open(char * fifo_filename);
void util_pipe_close();

typedef enum {
    DEBUGSS_STM_CLK,
    DSP_CORE_CLK,
    ARM_CORE_CLK
} clock_type_t;

long util_get_freq(clock_type_t clk_type);
#endif /* UTILITY_H */

