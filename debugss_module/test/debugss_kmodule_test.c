/*
 * debugss_kmodule_test.c
 *
 * Keystone DebugSS kernel module test application
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

/*****************************************************************************  
 *  Static Definitions and Initialization
 *****************************************************************************/
static int mem_fd = 0;                      /* Only need to open mem_fd once */

/***************************************************************************** 
 *  mem_map()
 *
 *  This function performs the physical memory mapping to this virtual space.
 *  
 *****************************************************************************/
void * mem_map(uint32_t map_addr, uint32_t map_size, int flags)
{
    int prot;
    if ((flags & O_RDWR) == O_RDWR) {
        prot = PROT_READ | PROT_WRITE;
    } else {
        prot = PROT_READ;
    }

    void * v_map_addr;

    v_map_addr = mmap(0, map_size, 
                                   prot, MAP_SHARED, mem_fd, 
                                   map_addr & ~(map_size -1));
    
    return v_map_addr;
}

/***************************************************************************** 
 *  cTools_memMap()
 *
 *  This function performs the physical memory mapping to this virtual space.
 *
 *  See mem_map() for details.
 *****************************************************************************/
void * cTools_memMap(uint32_t map_addr, uint32_t map_size)
{
    return mem_map(map_addr, map_size, O_RDWR);
}

/***************************************************************************** 
 *  cTools_memUnMap()
 *
 *  This function performs the virtual memory un-mapping.
 *
 *****************************************************************************/
void cTools_memUnMap(uint32_t map_addr, uint32_t map_size)
{
    int ret_val = munmap((void*)map_addr,
                         map_size);

    if (ret_val == -1) {
        return;
    }
}

void test1(void)
{
    void * virt_addr_ptr1; 
    void * virt_addr_ptr2;
    void * virt_addr_ptr3;

    //map CP tracer address space
	virt_addr_ptr1 = cTools_memMap(0x01D00000, 4096);

    if((void*)-1 == virt_addr_ptr1)
	{
		printf("DebugSS driver: test1 failed (CP Tracer mapping)\n");
        return;
	}

    if(0x4E891903 != *((uint32_t*)virt_addr_ptr1))
	{
		printf("DebugSS driver: test1 failed (CP Tracer mapping)\n");
        return;
	}

    //map STM CFG address space
	virt_addr_ptr2 = cTools_memMap(0x03018000, 4096);

    if((void*)-1 == virt_addr_ptr2)
	{
		printf("DebugSS driver: test1 failed (STM CFG mapping)\n");
	}

    if(0x50430400 != *((uint32_t*)virt_addr_ptr2))
	{
		printf("DebugSS driver: test1 failed (STM CFG mapping)\n");
        return;
	}

	//map DebugSS CFG address space
	virt_addr_ptr3 = cTools_memMap(0x03000000, 0x100000);

    if((void*)-1 == virt_addr_ptr3)
	{
		printf("DebugSS driver: test1 failed (DebugSS CFG mapping)\n");
        return;
	}

    if(0x50430400 != *((uint32_t*)((uint8_t*)virt_addr_ptr3+0x18000)))
	{
		printf("DebugSS driver: test1 failed (DebugSS CFG mapping)\n");
        return;
	}

    //unmap CP Tracer address space
    cTools_memUnMap((uint32_t)virt_addr_ptr1,4096);

    //unmap STM CFG address space
    cTools_memUnMap((uint32_t)virt_addr_ptr2,4096);

    //unmap DebugSS CFG address space
    cTools_memUnMap((uint32_t)virt_addr_ptr3,0x100000);

    //map CP tracer address space
	virt_addr_ptr1 = cTools_memMap(0x01D00000, 4096);

    if((void*)-1 == virt_addr_ptr1)
	{
		printf("DebugSS driver: test1 failed (CP Tracer mapping)\n");
        return;
	}

    if(0x4E891903 != *((uint32_t*)virt_addr_ptr1))
	{
		printf("DebugSS driver: test1 failed (CP Tracer mapping)\n");
        return;
	}

    //map STM CFG address space
	virt_addr_ptr2 = cTools_memMap(0x03018000, 4096);

    if((void*)-1 == virt_addr_ptr2)
	{
		printf("DebugSS driver: test1 failed (STM CFG mapping)\n");
        return;
	}

    if(0x50430400 != *((uint32_t*)virt_addr_ptr2))
	{
		printf("DebugSS driver: test1 failed (STM CFG mapping)\n");
        return;
	}

	//map DebugSS CFG address space
	virt_addr_ptr3 = cTools_memMap(0x03000000, 0x100000);

    if((void*)-1 == virt_addr_ptr3)
	{
		printf("DebugSS driver: test1 failed (DebugSS CFG mapping)\n");
        return;
	}

    if(0x50430400 != *((uint32_t*)((uint8_t*)virt_addr_ptr3+0x18000)))
	{
		printf("DebugSS driver: test1 failed (DebugSS CFG mapping)\n");
        return;
	}

    //unmap STM CFG address space
    cTools_memUnMap((uint32_t)virt_addr_ptr2,4096);

    //unmap DebugSS CFG address space
    cTools_memUnMap((uint32_t)virt_addr_ptr3,0x100000);

    //unmap CP Tracer address space
    cTools_memUnMap((uint32_t)virt_addr_ptr1,4096);

    printf("DebugSS driver: test1 Passed\n");
    
    return;
}

void test2(void)
{
    void * virt_addr_ptr; 

    //map invalid DebugSS address space
	virt_addr_ptr = cTools_memMap(0x02700000, 4096);

    if((void*)-1 != virt_addr_ptr)
	{
		printf("DebugSS driver: test2 failed (invalid DebugSS address mapping)\n");
        return;
	}

	//map 4K + first 4K DebugSS CFG address space
	virt_addr_ptr = cTools_memMap(0x02FFF000, 8192);

    if((void*)-1 != virt_addr_ptr)
	{
		printf("DebugSS driver: test2 failed (4K + 4K DebugSS CFG mapping)\n");
        return;
	}

	//map DebugSS CFG address space + 1MB
	virt_addr_ptr = cTools_memMap(0x03000000, 0x200000);

    if((void*)-1 != virt_addr_ptr)
	{
		printf("DebugSS driver: test2 failed (DebugSS CFG + 1MB mapping)\n");
        return;
	}

    printf("DebugSS driver: test2 Passed\n");
    
    return;
}

void test3(void)
{
    pid_t child_pid;

	void * virt_addr_ptr; 

    //map CP tracer address space
	virt_addr_ptr = cTools_memMap(0x01D00000, 4096);

    if((void*)-1 == virt_addr_ptr)
	{
		printf("DebugSS driver: test3 failed (CP Tracer mapping)\n");
        return;
	}

    if(0x4E891903 != *((uint32_t*)virt_addr_ptr))
	{
		printf("DebugSS driver: test3 failed (CP Tracer mapping)\n");
        return;
	}

    child_pid = fork();

    if(child_pid >= 0)
    {
        if(child_pid == 0) // child process
        {
            //map STM CFG address space
	        virt_addr_ptr = cTools_memMap(0x03018000, 4096);

            if((void*)-1 == virt_addr_ptr)
	        {
		        printf("DebugSS driver: test3 failed (STM CFG mapping)\n");
                return;
	        }

            if(0x50430400 != *((uint32_t*)virt_addr_ptr))
	        {
		        printf("DebugSS driver: test3 failed (STM CFG mapping)\n");
                return;
	        }

            //unmap STM CFG address space
            cTools_memUnMap((uint32_t)virt_addr_ptr,4096);            

            //map CP tracer address space
	        virt_addr_ptr = cTools_memMap(0x01D00000, 4096);

            if((void*)-1 != virt_addr_ptr)
	        {
		        printf("DebugSS driver: test3 failed (mapping memory which is already mapped by another process)\n");
	        }

            sleep(2);
        }
        else //Parent process
        {
            sleep(1);
            kill(child_pid, SIGTERM); //kill child process
        }
    }

    //unmap CP Tracer address space
    cTools_memUnMap((uint32_t)virt_addr_ptr,4096);

    printf("DebugSS driver: test3 Passed\n");
    
    return;
}

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
    
    if(mem_fd == 0) {
        mem_fd = open("/dev/debugss", O_RDWR | O_SYNC );
        //mem_fd = open("/dev/mem", flags | O_SYNC );
        if (mem_fd == -1) {
            printf("DebugSS driver: open failed\n");
            return;
        }
    }	

    test1();

    test2();

    test3();

    //Read target device name
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/class/misc/debugss/device_name", (char *)sysfs_read_buf);
    printf("Device Name: %s\n", sysfs_read_buf);

    //Read target mainpll clock
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/class/misc/debugss/mainpllclk", (char *)sysfs_read_buf);
    printf("Main PLL clock(Hz): %s\n", sysfs_read_buf);

    //Read target tetrispll clock
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/class/misc/debugss/tetrispllclk", (char *)sysfs_read_buf);
    printf("Tetris PLL clock(Hz): %s\n", sysfs_read_buf);

    //Read target debugss clock
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/class/misc/debugss/debugssclk", (char *)sysfs_read_buf);
    printf("DebugSS clock(Hz): %s\n", sysfs_read_buf);

    //Read target GEM Trace clock
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/class/misc/debugss/gemtraceclk", (char *)sysfs_read_buf);
    printf("GEM Trace clock(Hz): %s\n", sysfs_read_buf);

    //Read target TETB clock
    memset((char*)sysfs_read_buf,'\0',64);
    read_sysfs_entry("/sys/class/misc/debugss/tetbclk", (char *)sysfs_read_buf);
    printf("TETB clock(Hz): %s\n", sysfs_read_buf);

    close(mem_fd);
    mem_fd = 0;

	return;	
}
