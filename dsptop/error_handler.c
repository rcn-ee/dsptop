/*
 * error_handler.c
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
#include <string.h>
#include <errno.h>
#include "error_handler.h"
#include "debug_log.h"
#include "dsptop.h"
#include "io.h"

char * error_type[] = {
    [ERR_TYPE_SYSTEM] = "system error",
    [ERR_TYPE_LOCAL] = "internal error"
};

typedef enum {FATAL, NON_FATAL} error_prefix_t;

char * error_prefix[] = { 
    [FATAL] = "Fatal",
    [NON_FATAL] = "Non-Fatal"
};

/******************************************************************************
 * error_handler() 
 *
 *  - Print the error to stderr.
 *  - Log the error.
 *  - Clean-up if fatal and call exit().    
 *****************************************************************************/
#if DEBUG
void error_handler(error_type_t err_type, err_id_t err_id, char * err_context_msg,
                   const char * file, const char * func, int line )
#else
void error_handler(error_type_t err_type, err_id_t err_id, char * err_context_msg)
#endif
{
    LOGFUNC();

    char *msg_prefix;
    char *err_msg;
    char *sys_msg = "";
    char * fmt;

    LOGMSG("Error id is %d", err_id);

	msg_prefix = (error_info[err_id].fatal) ? error_prefix[FATAL]
                                            : error_prefix[NON_FATAL];
    
    if (err_context_msg == NULL) {
        err_msg = error_info[err_id].msg;
    } else {
        err_msg = err_context_msg;
    }

	if (err_type == ERR_TYPE_SYSTEM){
        sys_msg = strerror(errno);
    }

    if (error_info[err_id].fatal) {
        io_term();
    }

    if (err_msg != NULL) {
#if DEBUG
        fmt = "%s:%d:%s:%s %s: %s: %s: %s\n";
        fprintf(stderr, fmt, file, line, func, g_whoami, error_type[err_type], msg_prefix,
                sys_msg, err_msg);

        fmt = "%s:%d:%s:%s %s: %s: %s";
        LOGERR(fmt, file, line, func, error_type[err_type], msg_prefix, sys_msg, err_msg); 
#else
        fmt = "%s %s: %s: %s: %s\n";
        fprintf(stderr, fmt, g_whoami, error_type[err_type], msg_prefix, sys_msg, err_msg);

        fmt = "%s %s: %s: %s: %s";
        LOGERR(fmt, __func__, error_type[err_type], msg_prefix, sys_msg, err_msg);
#endif
   } else {
#if DEBUG
        fmt = "%s:%d:%s:%s %s: %s";
        LOGERR(fmt, file, line, func, error_type[err_type], msg_prefix, sys_msg );
#else
        fmt = "%s %s: %s: %s:";
        LOGERR(fmt, __func__, error_type[err_type], msg_prefix, sys_msg);        
#endif
   }


    if (error_info[err_id].fatal) {
        exit(-1);
    }
}

