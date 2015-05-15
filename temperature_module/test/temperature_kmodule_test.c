/*
 * temperature_kmodule_test.c
 *
 * Keystone temperature HWMON kernel module test application
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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>

void read_sysfs_entry(const char * sysfs_entry_path, char * sysfs_read_buf)
{
    int sysfs_fd = 0;
    char * sysfs_read_ptr;
    ssize_t read_byte_count;

    /* Read sysfs system clock entries */
    if(sysfs_fd == 0) {
        sysfs_fd = open(sysfs_entry_path, O_RDONLY);
        if (sysfs_fd == -1) {
            printf("DebugSS driver: open failed\n");
            return;
        }

        sysfs_read_ptr = sysfs_read_buf;
        
        while(1)
        {
            read_byte_count = read(sysfs_fd, (void*)sysfs_read_ptr, 1);

            if(0 == read_byte_count)
            {
                break;
            }
            else if(1 == read_byte_count)
            {
                sysfs_read_ptr++;
            }
            else
            {
                printf("DebugSS driver: read failed\n");
                return;
            }
        }
    }

    close(sysfs_fd);
    sysfs_fd = 0;
}

void main(void)
{

    char sysfs_read_buf[64];

    //Read kernel module name
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/name", (char *)sysfs_read_buf);
    printf("Kernel Module Name: %s\n", sysfs_read_buf);

    //Read SRSS0 status
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss0_status", (char *)sysfs_read_buf);
    printf("SRSS0 current status: %s\n", sysfs_read_buf);

    //Read SRSS0 temperature0
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss0_temp0", (char *)sysfs_read_buf);
    printf("SRSS0 Temperature0: %s\n", sysfs_read_buf);

    //Read SRSS0 temperature1
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss0_temp1", (char *)sysfs_read_buf);
    printf("SRSS0 Temperature1: %s\n", sysfs_read_buf);

    //Read SRSS1 status
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss1_status", (char *)sysfs_read_buf);
    printf("SRSS1 current status: %s\n", sysfs_read_buf);

    //Read SRSS1 temperature0
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss1_temp0", (char *)sysfs_read_buf);
    printf("SRSS1 Temperature0: %s\n", sysfs_read_buf);

    //Read SRSS1 temperature1
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/bus/platform/devices/2330000.srss/hwmon/hwmon0/srss1_temp1", (char *)sysfs_read_buf);
    printf("SRSS1 Temperature1: %s\n", sysfs_read_buf);

    return;	
}
