/*
 * map.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#ifndef LIB_BUILD
#include "debug_log.h"
#include "error_handler.h"
#include "error.h"
#endif

/*****************************************************************************  
 *  Static Definitions and Initialization
 *****************************************************************************/
static int mem_fd = 0;                      /* Only need to open mem_fd once */

/* Map Object */
struct map_element_t {
        void * v_map_addr;                  /* Virtual address */
        size_t v_map_size;                  /* Mapped size in bytes - may be whole number of pages */
        int    flags;                       /* Permission flags */
        uint32_t phy_addr;                  /* Used to request adj_v_addr */ 
        struct map_element_t * prev_map_element;
        struct map_element_t * next_map_element;
};

/* Map Object Link List head */
static struct map_element_t * map_table_head = NULL;

/***************************************************************************** 
 *  mem_map()
 *
 *  This function performs the physical memory mapping to this virtual space.
 *
 *  - Mapped memory blocks are kept in a link list, so first check the list
 *    if the map already exist. This is common if the client is re-started.
 *  - Open /dev/debugss if not already open.
 *  - Allocate space for the map element.
 *  - Adjust the size to a whole number of pages.
 *  - Map the physical address to the virtual space. Align the physical address
 *    to a page boundary if necessary.
 *  - Add map element to end of link list.
 *  
 *****************************************************************************/
void * mem_map(uint32_t map_addr, uint32_t map_size, int flags)
{
#ifndef LIB_BUILD
    LOGFUNC();
#endif
    struct map_element_t * map_element;
    uint32_t page_size = sysconf(_SC_PAGE_SIZE);

    struct map_element_t * this_map_element = map_table_head;

#if defined(DEBUG) && !defined(LIB_BUILD)
    if ((flags != O_RDONLY) && (flags != O_RDWR)) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);        
    }
#endif
    /* Check if map_addr already mapped. 
     * Could make this more general purpose by checking 
     * if the address is in a mapped range, but this works fine
     * as a simple check of debug ip mapping.
     */
    while (this_map_element != NULL) {

        if ((this_map_element->phy_addr == map_addr)
            && (this_map_element->v_map_size >= map_size)) {
#ifndef LIB_BUILD
            LOGMSG("%s:0x%x already mapped", __func__, map_addr);
#ifdef DEBUG
            if (this_map_element->flags != flags) {
               err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
            } 
#endif
#endif
            return this_map_element->v_map_addr;
        }
        this_map_element = this_map_element->next_map_element;
    }

    if(mem_fd == 0) {
#ifdef SUDO_BUILD
        mem_fd = open("/dev/mem", flags | O_SYNC );
#else
        mem_fd = open("/dev/debugss", flags | O_SYNC );
#endif
        if (mem_fd == -1) {
#ifndef LIB_BUILD
#ifdef SUDO_BUILD
            char * err_msg_p = (flags == O_RDONLY) ? "Can not open '/dev/mem' with O_RDONLY | O_SYNC attributes"
                                                  : "Can not open '/dev/mem' with O_RDWR | O_SYNC attributes";
#else
            char * err_msg_p = (flags == O_RDONLY) ? "Can not open '/dev/debugss' with O_RDONLY | O_SYNC attributes"
                                                  : "Can not open '/dev/debugss' with O_RDWR | O_SYNC attributes";
#endif
            err_handler(ERR_TYPE_SYSTEM, ERR_MMAP_OPEN, err_msg_p);
#else
            return (void *)-1;
#endif
        }
    }

    /* Allocate space for the map element */
    map_element = (struct map_element_t *)malloc(sizeof(struct map_element_t));
    if (map_element == NULL) {
#ifndef LIB_BUILD
       err_handler(ERR_TYPE_SYSTEM, ERR_MEM_ALLOC, NULL);
        /* Dummy return to keep code checker tool from
         * declaring an issue, err_handler will call exit.
         */
#endif
        return (void *)-1;
    }

    /* Note: the physical address (map_addr) may need to be aligned to a 
     *  PAGE_SIZE and map_size may need to be a multiple of PAGE_SIZE.
     *  This means the virtual address may need to be adjusted by the page mask
     *  to get the same physical address (adj_v_addr).
     */
    if ((map_size % page_size) != 0) {
        map_element->v_map_size = ((map_size / page_size) + 1) * page_size;
    } else {
        map_element->v_map_size = map_size;
    }

    int prot;
    if ((flags & O_RDWR) == O_RDWR) {
        prot = PROT_READ | PROT_WRITE;
    } else {
        prot = PROT_READ;
    }

    map_element->v_map_addr = mmap(0, map_element->v_map_size, 
                                   prot, MAP_SHARED, mem_fd, 
                                   map_addr & ~(map_element->v_map_size -1));

    mlock(map_element->v_map_addr, map_element->v_map_size);

    map_element->phy_addr = map_addr;
    map_element->flags = flags;

#ifndef LIB_BUILD
    LOGMSG("%s:phy addr: 0x%08x, phy size: %d", __func__, map_addr, map_size);
    if (prot == PROT_READ) {
        LOGMSG("%s:map read protected only", __func__);
    } else { 
        LOGMSG("%s:map both read and write protected", __func__);
    }
    LOGMSG("%s:mapping phy addr 0x%08x",__func__, 
            map_addr & ~(map_element->v_map_size -1)); 
    LOGMSG("%s:vir addr: 0x%08x, vir size: %d",__func__, 
            map_element->v_map_addr, map_element->v_map_size);
#endif
    if (map_element->v_map_addr == MAP_FAILED) {
        free(map_element);
        /* Fatal so error handler exits */ 
#ifndef LIB_BUILD
        char * msg = "Check for another instance of dsptop running, only one instance per device allowed."; 
        err_handler(ERR_TYPE_SYSTEM, ERR_MMAP_FAIL, msg);
        /* Dummy return to keep code checker tool from
         * declaring an issue, err_handler will call exit.
         */
#endif
        return (void *)-1;
    }

    /* Add map element to end of link list */
    map_element->next_map_element = NULL;
    if (map_table_head == NULL) {
        map_table_head = map_element;
        map_element->prev_map_element = NULL;
    } else {
        /* Search for the last element */
        struct map_element_t * this_map_element = map_table_head;
        while (this_map_element->next_map_element != NULL) {
            this_map_element = this_map_element->next_map_element;
        }

        this_map_element->next_map_element = map_element;
        map_element->prev_map_element = this_map_element;
    }

    return map_element->v_map_addr;
}

/***************************************************************************** 
 *  cTools_memMap()
 *
 *  This function performs the physical memory mapping to this virtual space.
 *
 *  See mem_map() for details.
 *  
 *  Note: This function is also used by ETBLib (which is statically linked
 *  with dsptop). For this reason the function prototype is required to be
 *  compatible with the cToolsHelper version that is used with ETBLib.
 *****************************************************************************/
void * cTools_memMap(uint32_t map_addr, uint32_t map_size)
{
    return mem_map(map_addr, map_size, O_RDWR);
}

/***************************************************************************** 
 *  mem_unmap()
 *
 *  This function performs the virtual memory un-mapping.
 *
 *  - Find the virtual address in the link list.
 *  - Unmap the virtual block.
 *  - Remove it from the link list.
 *  
 *****************************************************************************/
void mem_unmap(void * v_map_addr, uint32_t map_size)
{
#ifndef LIB_BUILD
    LOGFUNC();
#endif

    /* Search for the element */
    struct map_element_t * this_map_element = map_table_head;

    if (this_map_element == NULL) {
#ifndef LIB_BUILD
        LOGMSG("%s: map_table_haed is NULL", __func__);
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
        /* Dummy return to keep code checker tool from
         * declaring an issue, err_handler will call exit.
         */
#endif
        return;   
    }

    while (this_map_element->v_map_addr != v_map_addr) {
        this_map_element = this_map_element->next_map_element;
        if (this_map_element == NULL) break;
    }    

    if (this_map_element == NULL) {
#ifndef LIB_BUILD
        LOGMSG("%s: v_map_addr not found", __func__);
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
        /* Dummy return to keep code checker tool from
         * declaring an issue, err_handler will call exit.
         */
#endif
        return;
    }

    int ret_val = munmap(this_map_element->v_map_addr,
                         this_map_element->v_map_size);

    if (ret_val == (int)MAP_FAILED) {
       free(this_map_element);
#ifndef LIB_BUILD
       /* Fatal so error handler exits */ 
       err_handler(ERR_TYPE_SYSTEM, ERR_MUNMAP_FAIL, NULL);
#endif
       return;    
    }

    /* Only element in link list */
    if ((this_map_element->next_map_element == NULL)
        && (this_map_element->prev_map_element == NULL)) {

        map_table_head = NULL;
        close(mem_fd);
        mem_fd = 0;
    }

    /* First element in link list */
    if ((this_map_element->next_map_element != NULL)
        && (this_map_element->prev_map_element == NULL)) {

      map_table_head = this_map_element->next_map_element;
      map_table_head->prev_map_element = NULL; 
    }

    /* Last element in link list */
    if ((this_map_element->next_map_element == NULL)
        && (this_map_element->prev_map_element != NULL)) {

        this_map_element->prev_map_element->next_map_element = NULL;
    }

    /* Middle element in link list */
    if ((this_map_element->next_map_element != NULL)
        && (this_map_element->prev_map_element != NULL)) {

        this_map_element->prev_map_element->next_map_element = 
                                            this_map_element->next_map_element;
        this_map_element->next_map_element->prev_map_element = 
                                            this_map_element->prev_map_element;

    }

    free(this_map_element);

}
/***************************************************************************** 
 *  cTools_memUnMap()
 *
 *  This function performs the virtual memory un-mapping.
 *
 *  See mem_unmap() for details.
 *  
 *  Note: This function is also used by ETBLib (which is statically linked
 *  with dsptop). For this reason the function prototype is required to be
 *  compatible with the cToolsHelper version that is used with ETBLib.
 *****************************************************************************/
void cTools_memUnMap(void * v_map_addr, uint32_t map_size)
{
    mem_unmap(v_map_addr, map_size);
}
/***************************************************************************** 
 *  mem_is_mapped()
 *
 *  Return virtual version of address if mapped
 *
 *  - Find the physical address in the link list and return it's virtual address.
 *****************************************************************************/
void * mem_is_mapped(uint32_t addr, size_t byte_cnt, void ** v_start, size_t * len)
{
#ifndef LIB_BUILD
    LOGFUNC();
#endif

    uint32_t page_size = sysconf(_SC_PAGE_SIZE);
    uint32_t page_mask = page_size -1;    

    /*Search map table for match */
    struct map_element_t * this_map_element = map_table_head;
    while (this_map_element != NULL) {

        if ((this_map_element->phy_addr == (addr & ~(this_map_element->v_map_size - 1))) 
            && ((addr + byte_cnt - 1) <= (this_map_element->phy_addr 
            + this_map_element->v_map_size - 1))  ) {
#ifndef LIB_BUILD
            LOGMSG("%s:Map element found, returning address 0x%x", __func__, 
                     this_map_element->v_map_addr + (addr & page_mask));
#endif
            *v_start = this_map_element->v_map_addr;
            *len = this_map_element->v_map_size;
            return this_map_element->v_map_addr + (addr & page_mask);

        }
        this_map_element = this_map_element->next_map_element;
    }    
#if defined(DEBUG) && !defined(LIB_BUILD)
    err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
#endif
    return 0;
}
