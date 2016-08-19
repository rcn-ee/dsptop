/*
 * utility.c
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
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "error_handler.h"
#include "error.h"
#include "debug_log.h"
#include "utility.h"

static int fifo_fd = -1;

/* By contract with the debugSS driver, device_name cannot be more than 15 characters. */
static char device_name[16]; 

static char * keystone_filename_p[] = { 
#if SUDO_BUILD
    [DEBUGSS_STM_CLK] = "/sys/kernel/debug/clk/refclk-main/mainpllclk/mainmuxclk/chipclk1/chipclk13/debugss-trc/clk_rate",
    [DSP_CORE_CLK] = "/sys/kernel/debug/clk/refclk-main/mainpllclk/mainmuxclk/chipclk1/clk_rate",
    [ARM_CORE_CLK] = "/sys/kernel/debug/clk/refclk-main/mainpllclk/mainmuxclk/chipclk1/clk_rate"
#else
    [DEBUGSS_STM_CLK] = "/sys/class/misc/debugss/debugssclk",
    [DSP_CORE_CLK] = "/sys/class/misc/debugss/mainpllclk",
    [ARM_CORE_CLK] = "/sys/class/misc/debugss/armpllclk"
};
#endif    
static char * dra7xx_filename_p[] = { 
#if SUDO_BUILD
    [DEBUGSS_STM_CLK] = "/sys/kernel/debug/clk/refclk-main/mainpllclk/mainmuxclk/chipclk1/chipclk13/debugss-trc/clk_rate",
    [DSP_CORE_CLK] = NULL,
    [ARM_CORE_CLK] = NULL
#else
    [DEBUGSS_STM_CLK] = "/sys/class/misc/debugss/debugssclk",
    [DSP_CORE_CLK] = NULL,
    [ARM_CORE_CLK] = NULL
};
#endif    

static long clock_values[] = {
    [DEBUGSS_STM_CLK] = -1,
    [DSP_CORE_CLK] = -1,
    [ARM_CORE_CLK] = -1
};

typedef struct {
    char * device;
    char ** filename_p;
} device_table_t;

static device_table_t device_table[] = {
    {.device = "66AK2Hxx", .filename_p = keystone_filename_p}, 
    {.device = "TCI6630K2L", .filename_p = keystone_filename_p},   
    {.device = "66AK2Exx", .filename_p = keystone_filename_p},
    {.device = "DRA74x", .filename_p = dra7xx_filename_p},
    {.device = "DRA75x", .filename_p = dra7xx_filename_p},
    {.device = "OMAP572x", .filename_p = dra7xx_filename_p},
    {.device = "AM572x", .filename_p = dra7xx_filename_p},
    {.device = "Unknown", .filename_p = NULL}
};


/***************************************************************************** 
 *  util_form_filename()
 *
 *  Concatenate filename with ext and put result in buf. If filename + ext is
 *  to large to fit a fatal error is generated.
 * 
 *  - Get the etb buffer from the head of the queue.
 *  - Free the queue head.
 *  - Return NULL if no data on the queue.
 *****************************************************************************/

void util_gen_filename(char * buf, size_t buf_size, const char * file_name, const char * file_ext)
{
    LOGFUNC();
    LOGMSG("%s concatenate %s with %s", __func__, file_name, file_ext);

    size_t filename_len = strlen(file_name);
    size_t ext_len = strlen(file_ext);

    if ((filename_len + ext_len + 1) > buf_size) {
        char * msg = "Filename exceeds max characters";
        LOGMSG ("%s:%s for %s%s", __func__, msg, file_name, file_ext);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

    strncpy(buf, file_name, filename_len + 1);
    strncat (buf, file_ext, ext_len + 1);

    LOGMSG("%s result is %s ", __func__, buf); 

}

/***************************************************************************** 
 *  util_get_filesize()
 *
 *****************************************************************************/
size_t util_get_filesize(char * out_filename_p)
{
    LOGFUNC();
    struct stat file_stat;

    if(stat(out_filename_p, &file_stat) < 0) {
        err_handler(ERR_TYPE_SYSTEM, ERR_LOGFILE, NULL);
    }

    return file_stat.st_size;

}

/***************************************************************************** 
 *  util_get_timestring
 *
 *  Convert a timeval struct to a string (HH:MM:SEC:MSEC:USEC)
 *
 *  Return the number of characters written to time_buf_p
 *  Note: time_buf_size minumum 16 char + \0, so 17 characters.
 *
 *  Note: this gets called often in logging mode so no debug log info generated.
 *
 *****************************************************************************/
size_t util_get_timestring(char * time_buf_p, size_t time_buf_size, struct timeval * time_tv, timestring_resolution_t time_res)
{
    size_t chr_cnt = 0;

    long hours = time_tv->tv_sec/3600;
    long minutes = (time_tv->tv_sec - (hours * 3600))/60;
    long seconds = time_tv->tv_sec - (hours * 3600) - (minutes * 60);

    chr_cnt += snprintf(time_buf_p + chr_cnt, time_buf_size - chr_cnt, "%2.2ld", hours);
    chr_cnt += snprintf(time_buf_p + chr_cnt, time_buf_size - chr_cnt, ":%2.2ld", minutes);
    chr_cnt += snprintf(time_buf_p + chr_cnt, time_buf_size - chr_cnt, ":%2.2ld", seconds);
    if (time_res == TIMESTRING_SECOND) {
        return chr_cnt;
    }

    if (time_res == TIMESTRING_MILLISECOND) {
        chr_cnt += snprintf(time_buf_p + chr_cnt, time_buf_size - chr_cnt, ".%3.3ld", time_tv->tv_usec/1000);
        return chr_cnt;
    }
    chr_cnt += snprintf(time_buf_p + chr_cnt, time_buf_size - chr_cnt, ".%6.6ld", time_tv->tv_usec);    

    return chr_cnt; 
}

/******************************************************************************
 * util_get_device()
 *
 *****************************************************************************/

char * util_get_device()
{

    char * filename_p = "/sys/class/misc/debugss/device_name";

    FILE * devname_fp = fopen(filename_p, "r");
    if (devname_fp == NULL) {
        char * msg = "Can not open debugss module device_name sysfs entry for reading";
        LOGMSG("%s:%s: %s", __func__, msg, filename_p);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

    /* By contract with the debugSS driver, device_name cannot be more than 15 characters. */
    errno = 0;
    int error = fscanf(devname_fp, "%15s", device_name);

    fclose(devname_fp);
 
    if ((error == EOF) && (errno != 0)) {
        err_handler(ERR_TYPE_SYSTEM, ERR_DBGSS_SYSFILE, NULL);   
    }

    LOGMSG("%s: Target device is identified as %s", __func__, device_name);

    return device_name;

}
/***************************************************************************** 
 *  util_get_stmfreq()
 *
 *****************************************************************************/
long util_get_freq(clock_type_t clk_type)
{
    LOGFUNC()

    if (clock_values[clk_type] != -1) {
        return clock_values[clk_type];
    }

    FILE * clk_fp = NULL; 
    long value = 0;
    char * device_name = util_get_device();
    int num_elements = sizeof(device_table)/sizeof(device_table_t) - 1;

    int i;
    for(i = 0; i < num_elements; i++) {
        if (!strncmp(device_name, device_table[i].device, strlen(device_name))) {
            break;
        }
    }

    if (i == num_elements) {
        char * msg = "Device not supported";
        LOGMSG("%s:%s: %s", __func__, msg, device_name);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

    if (device_table[i].filename_p[clk_type] != NULL) {
        clk_fp = fopen(device_table[i].filename_p[clk_type], "r");
    }

    if (clk_fp == NULL) {
        char * msg = "Can not open debugss module clock sysfs entries for reading";
        LOGMSG("%s:%s: %s", __func__, msg, device_table[i].filename_p[clk_type]);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

    errno = 0;
    int error = fscanf(clk_fp, "%ld", &value);

    fclose(clk_fp);
 
    if ((error == EOF) && (errno != 0)) {
        err_handler(ERR_TYPE_SYSTEM, ERR_DBGSS_SYSFILE, NULL);   
    }

#if defined(DRA7xx)
    /* In case debugss fails to write STM frequency to the filesystem,       */
    /* use this value.  If debugss is fixed, the value here is not used.     */
    if (clk_type == DEBUGSS_STM_CLK && value == 0)  value = 188000000;
#endif

    LOGMSG("%s: %s is %d Hz", __func__, device_table[i].filename_p[clk_type], value);

    clock_values[clk_type] = value;

    return value;
}

/***************************************************************************** 
 *  util_pipe_write()
 *    
 *****************************************************************************/
void util_pipe_write(const char * wr_buf_p, size_t wr_bytecnt)
{
    LOGFUNC();

    if (fifo_fd < 0) {
        return;
    }

    while ( wr_bytecnt > 0) {
        size_t rc = write(fifo_fd, wr_buf_p, wr_bytecnt);

        if ((rc == -1) && (errno == EPIPE)) {
            err_handler(ERR_TYPE_SYSTEM, ERR_FIFO_PIPEBRK, NULL);            
        }
        if ((rc == -1) && (errno == EINTR)) {
            continue;
        }
        wr_buf_p += rc;
        wr_bytecnt -= rc;
    } 
}

/***************************************************************************** 
 *  util_pipe_write()
 *    
 *****************************************************************************/
void util_pipe_open(char * fifo_filename)
{
    LOGFUNC();

    if (mkfifo(fifo_filename, 0666) == -1) {
        if (errno != EEXIST) {
            err_handler(ERR_TYPE_SYSTEM, ERR_FIFO_MAKE, NULL);    
        }
        /* Error must be EEXIST so remove fifo file */
        LOGMSG("%s:%s exists", __func__, fifo_filename);
        if (remove(fifo_filename) != 0) {
           err_handler(ERR_TYPE_SYSTEM, ERR_FIFO_MAKE, NULL); 
        }
        /* Try to create fifo again */
        if (mkfifo(fifo_filename, 0666) == -1) {
            err_handler(ERR_TYPE_SYSTEM, ERR_FIFO_MAKE, NULL);
        }
    }    

    LOGMSG("%s:Opening %s for writing.", __func__, fifo_filename);

    if ((fifo_fd = open(fifo_filename, O_WRONLY)) == -1) {
        err_handler(ERR_TYPE_SYSTEM, ERR_FIFO_OPEN, NULL);
    }

    LOGMSG("%s:%s open for writing", __func__, fifo_filename);
}

/***************************************************************************** 
 *  util_pipe_close()
 *    
 *****************************************************************************/
void util_pipe_close()
{
    LOGFUNC();

    if (fifo_fd != -1) {
        close(fifo_fd);
        fifo_fd = -1;
    }
}

