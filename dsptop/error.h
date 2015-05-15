/*
 * error.h
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
#ifndef ERROR_H
#define ERROR_H

#include <stdbool.h>

struct error_info_t {
    char *msg;
    bool fatal;
};

typedef enum  { 
    ERR_NONE,
    ERR_MEM_ALLOC,
    ERR_CMD_PARSER,
    ERR_DEBUG_LOGFILE,
    ERR_IO,
    ERR_LOGFILE,
    ERR_DBGSS_SYSFILE,
    ERR_TMP_SYSFILE,
    ERR_INVALID_SIGNAL_CAUGHT,
    ERR_MMAP_OPEN,
    ERR_MMAP_FAIL,
    ERR_MUNMAP_FAIL,
    ERR_ETB_ISSUE, 
    ERR_ETB_OVERFLOW,
    ERR_ETB_ENABLED_STATE,
    ERR_FIFO_PIPEBRK,
    ERR_FIFO_MAKE,
    ERR_FIFO_OPEN,
    ERR_FATAL,
    ERR_NONFATAL,
    ERR_DEBUG
}err_id_t;

extern struct error_info_t error_info[];

#endif /*ERROR_H*/
