/*
 * debug_log.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include "dsptop.h"
#include "error_handler.h"
#include "debug_log.h"
#include "utility.h"

bool g_log_enable = false;
int  g_log_level = 0;

static FILE *logout;

#define time_buf_size 16 
static char time_buf[time_buf_size];
/***************************************************************************** 
 *  Internal Function Prototypes 
 *  - See Private Function section below for implementations )
 *****************************************************************************/
char * get_timestamp();

/***************************************************************************** 
 *  Public functions
 *****************************************************************************/
/******************************************************************************
 * log_handler() (for --logfile option)
 *
 *  - Enable logging to ctprof_log.txt and set log level.    
 *****************************************************************************/
void debug_log_int(int level)
{

    const size_t filename_max_size = 256;
    char log_filename[filename_max_size];

    if (g_log_enable == false) {

        g_log_level = level;

        util_gen_filename(log_filename, filename_max_size, g_whoami, "_log.txt");

        logout = fopen(log_filename,"w");
        if (logout == NULL) {
            err_handler(ERR_TYPE_LOCAL, ERR_DEBUG_LOGFILE, NULL);
            return;
        }
    }

    g_log_enable = true;

}

/******************************************************************************
 * log_msg()
 *
 *     - log message to logout with timestamp in milliseconds 
 *     - Display format " M|F|E H:M:S:MS | Function:Line: Message"
 *       where:
 *           M - message
 *           F - function start
 *           E - error      
 *****************************************************************************/
void debug_log_msg( log_type_t log_type, const char * format, ...)
{
    char * prefix = "   ";
	va_list marker;
	
    
    switch (log_type) {
    case LOG_MSG:
         prefix = " M ";
         break;
    case LOG_ERR:
         prefix = " E ";
         break;
    case LOG_FUNC:
         prefix = " F ";
    };

    va_start( marker, format);

    char * timestamp_p = get_timestamp();
    if (timestamp_p != NULL) {
        fprintf(logout, "%s %s | ", prefix, timestamp_p);
        vfprintf(logout, format, marker);
        fprintf(logout,"\n"); 
        va_end(marker);
        fflush(logout);
    }
}
/***************************************************************************** 
 *  Private functions
 *****************************************************************************/
/******************************************************************************
 * -get_timestamp
 *      Provide timestamp string in H:M:S:MS 
 *****************************************************************************/
char * get_timestamp()
{

    struct timeval tv;
    time_t curtime;
    int time_size = 0;

    gettimeofday(&tv, NULL);
    curtime = tv.tv_sec;
    struct tm * local_tm = localtime(&curtime);

    if (local_tm == NULL) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
        return NULL;
    }
    time_size = strftime(time_buf, time_buf_size, "%T", local_tm);

    snprintf(time_buf + time_size, time_buf_size, ":%03ld", tv.tv_usec/1000); 

    return time_buf;
}



