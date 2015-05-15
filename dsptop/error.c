/*
 * error.c
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

#include <stdlib.h>
#include "error.h"

struct error_info_t error_info[] = {
    [ERR_NONE] = {"No error", false},
    [ERR_MEM_ALLOC] = {"Memory Allocation Error", true},
    [ERR_CMD_PARSER] =    {"Invalid command option - try --help", true},
    [ERR_DEBUG_LOGFILE] = {"Can not open debug log file", false},
    [ERR_IO] = {"System error during io", true},
    [ERR_LOGFILE] = {"Error writing to log file", true},
    [ERR_DBGSS_SYSFILE] = {"Error accessing DebugSS sysfs file", true},
    [ERR_TMP_SYSFILE] = {"Error accessing Temperature sysfs file", true},
    [ERR_INVALID_SIGNAL_CAUGHT] = {"Invalid signal caught", true},
    [ERR_MMAP_OPEN] = {"Can not open '/dev/mem' with O_RDWR | O_SYNC attributes", true},
    [ERR_MMAP_FAIL] = {"Can not map region", true},
    [ERR_MUNMAP_FAIL] = {"Can not unmap region", true},
    [ERR_ETB_ISSUE] = {"ETB Library error - see log file for details", true},
    [ERR_ETB_OVERFLOW] = {"ETB overflow error", true},
    [ERR_ETB_ENABLED_STATE] = {"ETB enabled state error", true},
    [ERR_FIFO_PIPEBRK] = {"Broken Pipe error", true},
    [ERR_FIFO_MAKE] = {"FIFO file make error", true},
    [ERR_FIFO_OPEN] = {"FIFO file open error", true},   
    [ERR_FATAL] = {NULL, true},     /* Message provided from error source */
    [ERR_NONFATAL] = {NULL, false},  
    [ERR_DEBUG] = {"Software assert error", true}
};
