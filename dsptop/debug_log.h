/*
 * debug.h
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

#ifndef DEBUG_H
#define DEBUG_H

#include <stdbool.h>

extern bool g_log_enable;
extern int  g_log_level;

typedef enum {LOG_MSG, LOG_ERR, LOG_FUNC} log_type_t;

void debug_log_int(int level);
void debug_log_msg(log_type_t log_type, const char * format, ...);


#define IF_LOGGING_ENABLED if (g_log_enable) 

/* Stadard logging messages if log level set to 1 or 2*/
#define LOGFUNC() if ((g_log_enable) && (g_log_level > 0) && (g_log_level < 3)) { \
                           debug_log_msg(LOG_FUNC,"%s", __func__); \
                       } 

#define LOGMSG(...) if ((g_log_enable) && (g_log_level > 0) && (g_log_level < 3)) {\
                           debug_log_msg(LOG_MSG, __VA_ARGS__);   \
                       }

/* For Log level 2 add ulm decoding messages - this can make log file very large */
#define LOGMSG2(...) if ((g_log_enable) && (g_log_level == 2)) {\
                           debug_log_msg(LOG_MSG, __VA_ARGS__);   \
                       }

/* For log level 2 or 3 log last display values for each dsp.
 * Used for automated validation of Moving Average and Total Modes.
 */
#define LOGMSG3(...) if ((g_log_enable) && (g_log_level > 1)) {\
                           debug_log_msg(LOG_MSG, __VA_ARGS__);   \
                       }


#define LOGERR(...) if (g_log_enable) { \
                           debug_log_msg(LOG_ERR, __VA_ARGS__); \
                       } 
#endif /*DEBUG_H*/

