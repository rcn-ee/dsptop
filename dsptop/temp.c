/*
 * temp.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include "dsptop.h"
#include "debug_log.h"
#include "error_handler.h"
#include "error.h"
#include "utility.h"

/*****************************************************************************  
 *  Static Definitions and Initialization
 *****************************************************************************/
static bool temp_enabled = false;
static FILE * temp_fp = NULL;
static int8_t * temp_usage_update_p = NULL;
static int8_t * temp_usage_max_p = NULL;

#define NUM_SRSS_MODULES          (2)
#define NUM_TEMP_SENSORS          (2)
#define SRSS0                     (0)
#define SRSS1                     (1)
#define TEMP0                     (0)
#define TEMP1                     (1)

static char *status_filename_p[NUM_SRSS_MODULES] = {
    "/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss0_status",
    "/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss1_status"
};

static char *temp_filename_p[NUM_SRSS_MODULES][NUM_TEMP_SENSORS] = {
    {"/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss0_temp0",
    "/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss0_temp1"},
    {"/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss1_temp0",
    "/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss1_temp1"}
};

/***************************************************************************** 
 * Public Functions
 *
 *****************************************************************************/

/***************************************************************************** 
 * temp_init
 *
 * - Initialize device temperature reading
 * - If temp device opens ok, then set temp_usage_update_p with temp_update_p. 
 *****************************************************************************/

void temp_init(int8_t * temp_update_p, int8_t * temp_max_p)
{
    LOGFUNC()
#ifndef SUDO_BUILD
    /* By contract with the temperature driver, the device_name cannot be more than 15 characters */
    char temp_device_name[16];
    char * filename_p = "/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/name";
    FILE * temp_devname_fp;
    int error;
    char * msg;
#endif

    if (g_mva_test_mode != TEST_MODE_DISABLED) {
        temp_fp = fopen("temptest.txt", "r");
        if (temp_fp != NULL) {
            temp_enabled = true;
            temp_usage_update_p = temp_update_p;
            temp_usage_max_p = temp_max_p;
            * temp_usage_max_p = -127;
            LOGMSG("%s:Temperature device opened for reading", __func__);
        } else {
            LOGMSG("%s:Could not open temptest.txt for reading", __func__);
            LOGMSG("%s:System error %d:%s", __func__, errno, strerror(errno));
        }
    } else {
#ifdef SUDO_BUILD
            /* Return with temp_enabled is false */
            return;
#else
        temp_devname_fp = fopen(filename_p, "r");
        if (temp_devname_fp == NULL) {
            msg = "Can not open temperature module name sysfs entry for reading";
            LOGMSG("%s:%s: %s", __func__, msg, filename_p);
//            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            return;
        }

        errno = 0;
        error = fscanf(temp_devname_fp, "%15s", temp_device_name);

        fclose(temp_devname_fp);

        if ((error == EOF) && (errno != 0)) {
            err_handler(ERR_TYPE_SYSTEM, ERR_TMP_SYSFILE, NULL);
        }

        LOGMSG("%s:Temperature module name is %s", __func__, temp_device_name);

        if(!strncmp(temp_device_name, "SRSStempmod", strlen(temp_device_name))) {
            temp_enabled = true;
            temp_usage_update_p = temp_update_p;
            temp_usage_max_p = temp_max_p;
            * temp_usage_max_p = -127;
            LOGMSG("%s:Temperature module initialization successful", __func__);              
        } else {
            LOGMSG("%s:Temperature module initialization failed", __func__);
        }
#endif
    }
}

/***************************************************************************** 
 * temp_is_enabled
 *
 * - Return temperature enable state
 *****************************************************************************/
bool temp_is_enabled()
{
    return temp_enabled;
}

/***************************************************************************** 
 * getmax_srss_temp
 *
 * - Get max current temperature reported by SRSS 
 *****************************************************************************/
static void getmax_srss_temp(uint8_t srss_index, uint8_t temp_index, int *value_p)
{
    FILE * temp_sysfs_fp;
    /* By contract with the temperature driver, temperature status cannot be more than 15 characters */
    char temp_status[16];
    char *msg;
    int error;
    int temp_value;
    
    /* Get SRSS0 temperature monitor status */
    temp_sysfs_fp = fopen(status_filename_p[srss_index], "r");
    if (temp_sysfs_fp == NULL) {
        msg = "Can not open temperature module sysfs entry for reading";
        LOGMSG("%s:%s: %s", __func__, msg, status_filename_p[srss_index]);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

    errno = 0;
    error = fscanf(temp_sysfs_fp, "%15s", temp_status);

    fclose(temp_sysfs_fp);

    if ((error == EOF) && (errno != 0)) {
        err_handler(ERR_TYPE_SYSTEM, ERR_TMP_SYSFILE, NULL);   
    }

    LOGMSG("%s:SRSS%d current status is %s", __func__, srss_index, temp_status);

    if(!strncmp(temp_status,"temp_valid", strlen(temp_status))) {
        /* Read SRSS temperature */
        temp_sysfs_fp = fopen(temp_filename_p[srss_index][temp_index], "r");
        if (temp_sysfs_fp == NULL) {
            msg = "Can not open temperature module sysfs entry for reading";
            LOGMSG("%s:%s: %s", __func__, msg, temp_filename_p[srss_index][temp_index]);
            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
        }

        errno = 0;
        error = fscanf(temp_sysfs_fp, "%d", &temp_value);

        fclose(temp_sysfs_fp);

        if ((error == EOF) && (errno != 0)) {
            err_handler(ERR_TYPE_SYSTEM, ERR_TMP_SYSFILE, NULL);   
        }

        LOGMSG("%s: SRSS%d temperature%d is %d C", __func__, srss_index, temp_index, temp_value);
        
        /* Always report only the max temperature measured */
        if(temp_value > *value_p) {
            *value_p = temp_value;
        }
    }
}

/***************************************************************************** 
 * temp_get
 *
 * - Get current device temperature
 *****************************************************************************/
int8_t temp_get()
{
    LOGFUNC();
    int value = -127;
    int error;

    if (temp_enabled) {

        if (g_mva_test_mode != TEST_MODE_DISABLED) {
            if (temp_fp == NULL) {
                LOGMSG("%s:Temperature device is not opened", __func__);
                err_handler(ERR_TYPE_SYSTEM, ERR_DEBUG, NULL);
                /* Dummy return to keep code checker from declaring a defect,
                 * err_handler() will exit for fatal errors.
                 */
                return -127;
            }

            /* By contract with the temperature driver, the size of value is -127 to 127 degrees C. */            
            errno = 0;
            error = fscanf(temp_fp, "%d", &value);

            if ((error == EOF) && (errno != 0)) {
                err_handler(ERR_TYPE_SYSTEM, ERR_TMP_SYSFILE, NULL);   
            } 

            /* EOF should only be returned if reading data from a real file.
             * In this case seek to beginning of file and start over.
             */
            if ((error == EOF) && (errno == 0)) {
                fseek(temp_fp, 0, SEEK_SET);
                value = 0;
            }

        } else {
                /* Get current temperature from SRSS */
                getmax_srss_temp(SRSS0,TEMP0,&value);
                getmax_srss_temp(SRSS0,TEMP1,&value);
                getmax_srss_temp(SRSS1,TEMP0,&value);
                getmax_srss_temp(SRSS1,TEMP1,&value);
                
        } 

        LOGMSG("%s: device temperature is %d C", __func__, value);

        if (temp_usage_update_p != NULL) {
            *temp_usage_update_p = value;
        }

        if ((temp_usage_max_p != NULL) && (value > *temp_usage_max_p)) {
            *temp_usage_max_p = value;
        }

    } else {
            LOGMSG("%s: Temperature module is not enabled", __func__);
    }

    return (int8_t)value; 

}

/***************************************************************************** 
 * temp_term
 *
 * - Terminate device temperature reading
 *****************************************************************************/
void temp_term() 
{
    LOGFUNC();
    if (g_mva_test_mode != TEST_MODE_DISABLED) {
        if (temp_fp != NULL) {
            fclose(temp_fp);
        }
    }
}

