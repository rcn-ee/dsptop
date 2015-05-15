/*
 * ulm.c
 *
 * Usage & Load Monitor Implementation
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

#include "tiulm.h"
#include "stm.h"
#include "version.h"

#ifdef __linux
/***************************************************************************** 
 *  Static Declarations for Linux build
 *****************************************************************************/
#include "map.h"
static const char * ulm_statefmt_table[] = {
    [ULM_STATE_IDLE] = "Idle, task id %d, tracking id %d",
    [ULM_STATE_RUN] = "Running, task id %d, tracking id %d",
    [ULM_STATE_EXIT] = "Exit, task id %d, tracking id %d",
    [ULM_OMP_EVENT_PARALLEL_START_ENTER] = "OpenMP Event Parallel Start Enter, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_PARALLEL_START_EXIT] = "OpenMP Event Parallel Start Exit, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_PARALLEL_END_ENTER] = "OpenMP Event Parallel End Enter, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_PARALLEL_END_EXIT] = "OpenMP Event Parallel End Exit, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_TASK_GEN] = "OpenMP Event Task Gen, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_TASK_START] = "OpenMP Event Task Start, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_TASK_END] = "OpenMP Event Task End, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_IMPLICIT_TASK_START] = "OpenMP Event Implicit Task Start, task id %d, tracking id 0x%08x",
    [ULM_OMP_EVENT_IMPLICIT_TASK_END] = "OpenMP Event Implicit Task End, task id %d, tracking id 0x%08x",
    [ULM_OCL_EXIT] = "OpenCL Monitor Exit, Kernel id %d, Work Group id %d",
    [ULM_OCL_NDR_OVERHEAD] = "OpenCL NDR Overhead, Kernel id %d, Work Group id %d",
    [ULM_OCL_NDR_KERNEL_START] = "OpenCL NDR Kernel Start, Kernel id %d, Work Group id %d",
    [ULM_OCL_NDR_KERNEL_COMPLETE] = "OpenCL NDR Kernel Complete, Kernel id %d, Work Group id %d",
    [ULM_OCL_NDR_CACHE_COHERENCE_COMPLETE] = "OpenCL NDR Cache Coherence Complete, Kernel id %d, Work Group id %d",
    [ULM_OCL_OOT_OVERHEAD] = "OpenCL OOT Overhead, Kernel id %d, Work Group id %d",
    [ULM_OCL_OOT_KERNEL_START] = "OpenCL OOT Kernel Start, Kernel id %d, Work Group id %d",
    [ULM_OCL_OOT_KERNEL_COMPLETE] = "OpenCL OOT Kernel Complete, Kernel id %d, Work Group id %d",
    [ULM_OCL_OOT_CACHE_COHERENCE_COMPLETE] = "OpenCL OOT Cache Coherence Complete, Kernel id %d, Work Group id %d",
    [ULM_OCL_IOT_OVERHEAD] = "OpenCL IOT Overhead, Kernel id %d, Work Group id %d",
    [ULM_OCL_IOT_KERNEL_START] = "OpenCL IOT Kernel Start, Kernel id %d, Work Group id %d",
    [ULM_OCL_IOT_KERNEL_COMPLETE] = "OpenCL IOT Kernel Complete, Kernel id %d, Work Group id %d",
    [ULM_OCL_IOT_CACHE_COHERENCE_COMPLETE] = "OpenCL IOT Cache Coherence Complete, Kernel id %d, Work Group id %d" 
};

static const char * ulm_statemsg_table[] = {
    [ULM_STATE_IDLE] = "Idle",
    [ULM_STATE_RUN] = "Running",
    [ULM_STATE_EXIT] = "Exit",
    [ULM_OMP_EVENT_PARALLEL_START_ENTER] = "OpenMP Event Parallel Start Enter",
    [ULM_OMP_EVENT_PARALLEL_START_EXIT] = "OpenMP Event Parallel Start Exit",
    [ULM_OMP_EVENT_PARALLEL_END_ENTER] = "OpenMP Event Parallel End Enter",
    [ULM_OMP_EVENT_PARALLEL_END_EXIT] = "OpenMP Event Parallel End Exit",
    [ULM_OMP_EVENT_TASK_GEN] = "OpenMP Event Task Gen",
    [ULM_OMP_EVENT_TASK_START] = "OpenMP Event Task Start",
    [ULM_OMP_EVENT_TASK_END] = "OpenMP Event Task End",
    [ULM_OMP_EVENT_IMPLICIT_TASK_START] = "OpenMP Event Implicit Task Start",
    [ULM_OMP_EVENT_IMPLICIT_TASK_END] = "OpenMP Event Implicit Task End",
    [ULM_OCL_EXIT] = "OpenCL Monitor Exit",
    [ULM_OCL_NDR_OVERHEAD] = "OpenCL NDR Overhead",
    [ULM_OCL_NDR_KERNEL_START] = "OpenCL NDR Kernel Start",
    [ULM_OCL_NDR_KERNEL_COMPLETE] = "OpenCL NDR Kernel Complete",
    [ULM_OCL_NDR_CACHE_COHERENCE_COMPLETE] = "OpenCL NDR Cache Coherence Complete",
    [ULM_OCL_OOT_OVERHEAD] = "OpenCL OOT Overhead",
    [ULM_OCL_OOT_KERNEL_START] = "OpenCL OOT Kernel Start",
    [ULM_OCL_OOT_KERNEL_COMPLETE] = "OpenCL OOT Kernel Complete",
    [ULM_OCL_OOT_CACHE_COHERENCE_COMPLETE] = "OpenCL OOT Cache Coherence Complete",
    [ULM_OCL_IOT_OVERHEAD] = "OpenCL IOT Overhead",
    [ULM_OCL_IOT_KERNEL_START] = "OpenCL IOT Kernel Start",
    [ULM_OCL_IOT_KERNEL_COMPLETE] = "OpenCL IOT Kernel Complete",
    [ULM_OCL_IOT_CACHE_COHERENCE_COMPLETE] = "OpenCL IOT Cache Coherence Complete"
};

/* Each state has a coresponding ULM_STATE_IDLE or ULM_STATE_RUN */
static const ulm_state_t ulm_state_table[] = {
    [ULM_STATE_IDLE] = ULM_STATE_IDLE,
    [ULM_STATE_RUN] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_PARALLEL_START_ENTER] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_PARALLEL_START_EXIT] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_PARALLEL_END_ENTER] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_PARALLEL_END_EXIT] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_TASK_GEN] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_TASK_START] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_TASK_END] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_IMPLICIT_TASK_START] = ULM_STATE_RUN,
    [ULM_OMP_EVENT_IMPLICIT_TASK_END] = ULM_STATE_IDLE,
    [ULM_OCL_EXIT] = ULM_STATE_EXIT,
    [ULM_OCL_NDR_OVERHEAD] = ULM_STATE_RUN,
    [ULM_OCL_NDR_KERNEL_START] = ULM_STATE_RUN,
    [ULM_OCL_NDR_KERNEL_COMPLETE] = ULM_STATE_IDLE,
    [ULM_OCL_NDR_CACHE_COHERENCE_COMPLETE] = ULM_STATE_IDLE,
    [ULM_OCL_OOT_OVERHEAD] = ULM_STATE_RUN,
    [ULM_OCL_OOT_KERNEL_START] = ULM_STATE_RUN,
    [ULM_OCL_OOT_KERNEL_COMPLETE] = ULM_STATE_IDLE,
    [ULM_OCL_OOT_CACHE_COHERENCE_COMPLETE] = ULM_STATE_IDLE,
    [ULM_OCL_IOT_OVERHEAD] = ULM_STATE_RUN,
    [ULM_OCL_IOT_KERNEL_START] = ULM_STATE_RUN,
    [ULM_OCL_IOT_KERNEL_COMPLETE] = ULM_STATE_IDLE,
    [ULM_OCL_IOT_CACHE_COHERENCE_COMPLETE] = ULM_STATE_IDLE,
    [ULM_STATE_EXIT] = ULM_STATE_EXIT
};

static uint32_t getSyncCount(uint32_t traceBufSize );

static void * stm_vc_base_addr = (void *)-1;    /* STM virtual controller base address */ 
static void * stm_vt_base_addr = (void *)-1;    /* STM virtual transport base address
                                                 * In the Linux case must call ulm_config
                                                 * first to initialize this value.
                                                 */
static void * cstf_vc_base_addr = (void *)-1;   /* CSTF virtual controller base address */

static uint32_t stm_vc_size = 0;
static uint32_t stm_vt_size = 0;
static uint32_t cstf_vc_size = 0;

static int stp_version = 0;

#define DNUM 0
#define MAX_NUM_CORES 1
static uint8_t stm_sequence_id[MAX_NUM_CORES] = {0};

static bool ulm_test_mode = false;
#else
/***************************************************************************** 
 *  Static Declarations for DSP builds
 *****************************************************************************/
#ifdef DRA7xx
#define DSP_SYS_HWINFO  (0x01D00004)
#define DNUM ((*((uint32_t *)DSP_SYS_HWINFO)) & 0xf)

#define MAX_NUM_CORES 2
#else /* Assume standard C66 core with DNUM supported */
extern cregister volatile unsigned int DNUM;

#define MAX_NUM_CORES 8
#endif

static uint8_t stm_sequence_id[MAX_NUM_CORES] = {0};

static void * stm_vt_base_addr = (uint32_t *)STM_XPORT_BASE_ADDR1;
#endif

/***************************************************************************** 
 *  Static Declarations for Linux and DSP builds
 *****************************************************************************/


static inline uint32_t get_port_addr(uint8_t ch);
static inline uint32_t get_timestamp_addr(uint8_t ch);

#define reg_rd8(addr) (*(volatile uint8_t *)addr)
#define reg_wr8(addr,value) (*(volatile uint8_t *)addr = (uint8_t)value)

#define reg_rd16(addr) (*(volatile uint16_t *)addr)
#define reg_wr16(addr,value) (*(volatile uint16_t *)addr = (uint16_t)value)

#define reg_rd32(addr) (*(volatile uint32_t *)addr)
#define reg_wr32(addr,value) (*(volatile uint32_t *)addr = (uint32_t)value)
/***************************************************************************** 
 *  Public Message Transport Functions
 *****************************************************************************/

void ulm_get_version(uint32_t * major, uint32_t * minor, uint32_t * patch)
{
    *major = MAJOR_VERSION;
    *minor = MINOR_VERSION;
    *patch = PATCH_VERSION;
}
/***************************************************************************** 
 *  ulm_put_state()
 *
 *  Write a ULM state to the STM Transport.
 * 
 *****************************************************************************/
void ulm_put_state(ulm_state_t state) 
{
#ifdef __linux
	if ((void *)-1 == stm_vt_base_addr) {
         return;
	}
#endif

    uint32_t stm_ts_port = get_timestamp_addr(state);

    reg_wr8((uint8_t *)stm_ts_port, stm_sequence_id[DNUM]++);
}

/***************************************************************************** 
 *  ulm_put_statemsg()
 *
 *  Write a ULM state message to the STM Transport.
 * 
 *****************************************************************************/
void ulm_put_statemsg(ulm_state_t state, uint32_t taskid, uint32_t value)
{
#ifdef __linux
	if ((void *)-1 == stm_vt_base_addr) {
         return;
	}
#endif

    uint32_t stm_port = get_port_addr(state);
    uint32_t stm_ts_port = get_timestamp_addr(state);

    reg_wr8((uint8_t *)stm_port, stm_sequence_id[DNUM]++);
    reg_wr32((uint32_t *)stm_port, taskid);
    reg_wr32((uint32_t *)stm_ts_port, value);

}

/***************************************************************************** 
 *  ulm_put_mem()
 *
 *  Write memory state to the STM transport. 
 *
 *  Note: percent_free is the fractional percentage (not multiplied by 100),
 *        so for 100% percent_free is 1.0; for 90.3 precent_free is .903.
 *
 *  - form a single uint32_t message where:
 *    Bits 0:9 Integer percent_free * 1000 (preserving 1 decimal point of precision)
 *    Bits 10:28 Number of 64K Byte Blocks of memory
 *****************************************************************************/
void ulm_put_mem(ulm_mem_t mem, uint32_t num_64kb_blocks, float frac_percent_free) 
{
#ifdef __linux
	if ((void *)-1 == stm_vt_base_addr ) {
         return;
	}
#endif
    
    uint32_t msg = frac_percent_free * 1000.0;
    msg &= 0x3FF;
    msg |= ((num_64kb_blocks & 0x3FFFF) << 10);

    uint32_t stm_ts_port = get_timestamp_addr(mem);

    reg_wr32((uint32_t *)stm_ts_port, msg);

}

/***************************************************************************** 
 *  ulm_put_sync()
 *
 *  This function is used in two cases:
 *  - As a periodic message to keep the STM Timestamp counter from saturating. 
 *  - As a flush message used to take care of an STM issue with the last
 *    message sent from the DSP not getting completely transmitted to the ETB.
 *
 *****************************************************************************/
void ulm_put_sync() 
{
#ifdef __linux
	if ((void *)-1 == stm_vt_base_addr ) {
         return;
	}
#endif

    uint32_t stm_ts_port = get_timestamp_addr(ULM_SYNC);

    reg_wr32((uint32_t *)stm_ts_port, 0);

}

/***************************************************************************** 
 *  ulm_put_temp()
 *
 *  Write temperature to the STM transport.
 *
 *  - form a single int8_t message where:
 *    Temperature is in the range -40 to 125 Degrees C. The temperature is
 *    in two's compliment format.
 *****************************************************************************/
void ulm_put_temp(int8_t temp) 
{
#ifdef __linux
	if ((void *)-1 == stm_vt_base_addr) {
         return;
	}
#endif

    uint32_t stm_ts_port = get_timestamp_addr(ULM_TEMP);

    reg_wr8((uint8_t *)stm_ts_port, temp); 

}

/***************************************************************************** 
 *  Private Message Transport Functions
 *****************************************************************************/
static inline uint32_t get_port_addr(uint8_t ch)
{
#if defined(__linux) && !defined(XPORT_ONLY)
    /* Adjust the CH to the start of the mapped space */
    if (!ulm_test_mode) {
        ch -= CH_START_DSPTOP;
    }
#endif
	return (uint32_t)stm_vt_base_addr + (STM_CHAN_RESOLUTION * ch);
}
static inline uint32_t get_timestamp_addr(uint8_t ch)
{
#if defined(__linux) && !defined(XPORT_ONLY)
    /* Adjust the CH to the start of the mapped space */
    if (!ulm_test_mode) {
        ch -= CH_START_DSPTOP;
    }
#endif  
	return (uint32_t)stm_vt_base_addr + (STM_CHAN_RESOLUTION * ch) + (STM_CHAN_RESOLUTION/2);
}


#ifdef __linux
/***************************************************************************** 
 *  Public configuration, control, and decode functions for Host only 
 *****************************************************************************/


/***************************************************************************** 
 *  Public Functions
 *****************************************************************************/

/***************************************************************************** 
 *  ulm_test_map()
 *
 *  Configure the STM unit for ATB Export
 *
 *****************************************************************************/ 
ulm_error_t ulm_test_map()
{
	/* Map all the physical STM transport register base address space to a virtual address */
    stm_vt_size = STM_CH_NUM_MIPI * STM_CHAN_RESOLUTION * 2;
	stm_vt_base_addr = cTools_memMap(STM_XPORT_BASE_ADDR1, stm_vt_size);
	if ( (void *)-1 == stm_vt_base_addr ){
         return ULM_ERR_STM_MMAP;
	}

    ulm_test_mode = true;
    return ULM_SUCCESS;
}

/***************************************************************************** 
 *  ulm_config()
 *
 *  Configure the STM unit for ATB Export
 *
 *****************************************************************************/ 
ulm_error_t ulm_config()
{
    stm_vt_size = STM_CH_NUM_MIPI * STM_CHAN_RESOLUTION;
#ifdef XPORT_ONLY
	/* Map the 1st half of physical STM transport register base address space to a virtual address */
	stm_vt_base_addr = cTools_memMap(STM_XPORT_BASE_ADDR1, stm_vt_size);
#else
    if (!ulm_test_mode) {
	    /* Map the 2nd half of physical STM transport register base address space to a virtual address */
	    stm_vt_base_addr = cTools_memMap(STM_XPORT_BASE_ADDR2, stm_vt_size);
	    if ( (void *)-1 == stm_vt_base_addr ){
             return ULM_ERR_STM_MMAP;
	    }
    }

#ifdef CSTF_DSS_BASE_ADDR
	/* Map the physical STM control register base address to a virtual address */
    cstf_vc_size = CSTF_CNTLREG_BLKSIZE;
	cstf_vc_base_addr = cTools_memMap(CSTF_DSS_BASE_ADDR, cstf_vc_size);

	if ( (void *)-1 == cstf_vc_base_addr ){
         return ULM_ERR_CSTF_MMAP;
	}

	/* Unlock and program CSTF module */
	reg_wr32(CSTF_DSS_LOCK(cstf_vc_base_addr), CS_UNLOCK_VALUE);
    reg_wr32(CSTF_DSS_CTRL(cstf_vc_base_addr), CSTF_DSS_CTRL_CTSTM);
#endif

	/* Map the physical STM control register base address to a virtual address */
    stm_vc_size = STM_CNTLREG_BLKSIZE;
	stm_vc_base_addr = cTools_memMap(STM_CONFIG_BASE_ADDR, stm_vc_size);
	if ( (void *)-1 == stm_vc_base_addr ){
         return ULM_ERR_STM_MMAP;
	}

	int claimRetry = STM_CLAIM_RETRY;

	/* If the Debugger already owns the STM unit then exit (can rely on Debugger to configure properly). */
	uint32_t swmctrl0_reg = reg_rd32(STM_MIPI_SWMSTCNTL0(stm_vc_base_addr));
	if (   (swmctrl0_reg & (OWNERSHIP_MASK | CURRENT_OWNER_MASK)) == (MOD_ENABLED | CURRENT_DEBUG_OWNS)
		|| (swmctrl0_reg & (OWNERSHIP_MASK | CURRENT_OWNER_MASK)) == (MOD_CLAIMED | CURRENT_DEBUG_OWNS))
	{
		return ULM_SUCCESS;
	}

	/* Unlock STM module */
	reg_wr32(STM_CS_LOCK(stm_vc_base_addr), CS_UNLOCK_VALUE);

	/* Claim it for the Application but leave it in Debug override
	 * mode allowing the Debugger to take control of it at any time.
     */
	reg_wr32(STM_MIPI_SWMSTCNTL0(stm_vc_base_addr), 0);
	reg_wr32(STM_MIPI_SWMSTCNTL0(stm_vc_base_addr), MOD_CLAIMED | DEBUG_OVERRIDE);

	/* Check the claim bit or already enabled and application owns the unit */
	do {
		swmctrl0_reg = reg_rd32(STM_MIPI_SWMSTCNTL0(stm_vc_base_addr));
	} while (  ((swmctrl0_reg & (OWNERSHIP_MASK | CURRENT_OWNER_MASK)) != (MOD_CLAIMED | CURRENT_APP_OWNS))
			&& ((swmctrl0_reg & (OWNERSHIP_MASK | CURRENT_OWNER_MASK)) != (MOD_ENABLED | CURRENT_APP_OWNS))
			&& ( 0 != claimRetry--) );

	if ( 0 >= claimRetry ) {
		return ULM_ERR_STM_OWNERSHIP_NOT_GRANTED;
	}

	/* Enable Masters */
	reg_wr32(STM_MIPI_SWMSTCNTL1(stm_vc_base_addr), STM_SW_MASTER_MAP);
	reg_wr32(STM_MIPI_SWMSTCNTL2(stm_vc_base_addr), STM_SW_MASTER_MASK);
	reg_wr32(STM_MIPI_SWMSTCNTL3(stm_vc_base_addr), 0x07070707);
	reg_wr32(STM_MIPI_SWMSTCNTL4(stm_vc_base_addr), 0x07070707);
	reg_wr32(STM_MIPI_HWMSTCNTL(stm_vc_base_addr), STM_HW_MASTER_MAP);

	/* Set STM PTI output to gated mode, 4 bit data and dual edge clocking */
	reg_wr32(STM_MIPI_PTICNFG(stm_vc_base_addr), 0x000000A0);
	/*Set STM master ID generation frequency for HW messages - not used for any SW messages */
	reg_wr32(STM_MIPI_PTICNTDOWN(stm_vc_base_addr), 0xFC);

    ulm_enable();

	/* Enable the STM module to export data */
	reg_wr32(STM_MIPI_SWMSTCNTL0(stm_vc_base_addr), MOD_ENABLED | STM_TRACE_EN);

#endif

    return ULM_SUCCESS;
}

/***************************************************************************** 
 *  ulm_term()
 *
 *  Release STM port address mapping
 *
 *****************************************************************************/ 
void ulm_term()
{
    if (stm_vt_base_addr != (void *)-1) {
	    cTools_memUnMap(stm_vt_base_addr, stm_vt_size);
        stm_vt_base_addr = (void *)-1;
    }

    if (stm_vc_base_addr != (void *)-1) {
        cTools_memUnMap(stm_vc_base_addr, stm_vc_size);
        stm_vc_base_addr = (void *)-1;
    }

#ifdef CSTF_DSS_BASE_ADDR
    if (cstf_vc_base_addr != (void *)-1) {
        cTools_memUnMap(cstf_vc_base_addr, cstf_vc_size);
        cstf_vc_base_addr = (void *)-1;
    }
#endif 
}

/***************************************************************************** 
 *  ulm_flush()
 *
 *  Flush the STM unit
 * 
 *****************************************************************************/ 
ulm_error_t ulm_flush()
{
    ulm_error_t retvalue = ULM_SUCCESS;
    int retry = STM_FLUSH_RETRY;

	if ( NULL == stm_vc_base_addr ){
         return ULM_ERR_STM_MMAP;
	}

	/* Wait for FIFO Empty bit to be set */
	while ((reg_rd32(STM_MIPI_SYSSTAT(stm_vc_base_addr)) & STM_FIFO_EMPTY)  != STM_FIFO_EMPTY){
		if (0 == --retry) break;
	}

    if (retry == 0) {
        retvalue = ULM_ERR_STM_FLUSH_FAILED;
    }

    return retvalue;
}

/***************************************************************************** 
 *  ulm_enable()
 *
 *  Enable the STM unit for ATB Export (does not enable the STM module)
 * 
 *****************************************************************************/
ulm_error_t ulm_enable()
{

	if ( NULL == stm_vc_base_addr ){
         return ULM_ERR_STM_MMAP;
	}

	if ( STM_MAJOR_STM2_0 > ( reg_rd32(STM_MIPI_ID_REG(stm_vc_base_addr)) & STM_MAJOR_MASK ) )
	{
		/* MIPI STM 1.0 */
        stp_version = 1;
		/* Enable ATB interface for ETB. Repeat master ID every 8 x 8 instrumentation access and
		 * repeat channel ID after F x 8 instrumentation access from master.
		 * This is needed to optimize ETB buffer in circular mode.
         */
		reg_wr32(STM_MIPI_ATBCNFG(stm_vc_base_addr), 0x0000F800);
		reg_wr32(STM_MIPI_ATBCNFG(stm_vc_base_addr), 0x0001F800);
	}
	else {
		/* MIPI STM 2.0 */
        stp_version = 2;
		/* Enable ATB interface for ETB. Since STP 2.0 ASYNC being used, the Master
		 * and Channel repeats can be set to 0. The ASYNC packet will provide new
		 * masters and channel numbers.
         */
		reg_wr32(STM_MIPI_ATBCNFG(stm_vc_base_addr), 0x0);

		/* Set the ATB ID */
		reg_wr32(STM_MIPI_ATBID(stm_vc_base_addr), STM_ATB_ID_MIPI);

		/* Setup the Sync counter */
		reg_wr32(STM_MIPI_ATBSYNCCNT(stm_vc_base_addr), getSyncCount(MIPI_STM_TBR_SIZE));

		/* Enable the ATB port */
		reg_wr32(STM_MIPI_ATBCNFG(stm_vc_base_addr), 0x00010000);
	}

    return ULM_SUCCESS;
}

/***************************************************************************** 
 *  ulm_disable()
 *
 *  Disable the STM unit for ATB Export (does not disable the STM module)
 * 
 *****************************************************************************/
ulm_error_t ulm_disable()
{
	if ( NULL == stm_vc_base_addr ){
         return ULM_ERR_STM_MMAP;
	}

    reg_wr32(STM_MIPI_ATBCNFG(stm_vc_base_addr), 0x0);

    return ULM_SUCCESS;
}

/***************************************************************************** 
 *  ulm_get_runstate()
 *
 *  Return either ULM_STATE_IDLE or ULM_STATE_RUN
 * 
 *****************************************************************************/
ulm_state_t ulm_get_runstate(ulm_state_t state)
{
    return ulm_state_table[state];
}

/***************************************************************************** 
 *  ulm_get_statefmt()
 *
 *  Get the state format string
 * 
 *****************************************************************************/
const char * ulm_get_statefmt(ulm_state_t state)
{  
    if (state < ULM_STATE_LAST) {
        return ulm_statefmt_table[state];
    } else {
        return NULL;
    }
}

/***************************************************************************** 
 *  ulm_get_statemsg()
 *
 *  Get the state message string
 * 
 *****************************************************************************/
const char * ulm_get_statemsg(ulm_state_t state)
{
    if (state < ULM_STATE_LAST) {
        return ulm_statemsg_table[state];
    } else {
        return NULL;
    }   
}

/***************************************************************************** 
 *  ulm_get_dcminfo()
 *
 *  Get the dcm information that ULMLib keeps
 * 
 *****************************************************************************/
void ulm_get_dcminfo(ulm_dcm_info_t  * dcm_info_p)
{
    if (dcm_info_p == NULL) {
        return;
	}

    dcm_info_p->stp_version = stp_version;

    if (stp_version == 1){

    	dcm_info_p->stm_data_flip = 1;
    	uint32_t value = reg_rd32(STM_MIPI_ATBHEADPTR(stm_vc_base_addr));

        dcm_info_p->atb_head_present_0 = (value & 0x8) >> 3;
        dcm_info_p->atb_head_pointer_0 = (value & 0x7);
        dcm_info_p->atb_head_present_1 = (value & 0x80) >> 7;
        dcm_info_p->atb_head_pointer_1 = (value & 0x70) >> 4;
    	dcm_info_p->atb_head_required = true;

        dcm_info_p->using_twp = false;

    }
    else {
    	dcm_info_p->stm_data_flip = 0;
        dcm_info_p->atb_head_present_0 = 0;
        dcm_info_p->atb_head_pointer_0 = 0;
        dcm_info_p->atb_head_present_1 = 0;
        dcm_info_p->atb_head_pointer_1 = 0;
    	dcm_info_p->atb_head_required = false;

        dcm_info_p->using_twp = true;
        dcm_info_p->atb_id = STM_ATB_ID_MIPI;
    }
}
/***************************************************************************** 
 *  Private Functions
 *****************************************************************************/

/* STPv2 SYNC COUNT table */
static uint32_t sync_count_table[] = {
    0,     /* eBUFSIZE_0  */
	245,   /* eBUFSIZE_4K */
	355,   /* eBUFSIZE_8K */
	510,   /* eBUFSIZE_16K */
	730,   /* eBUFSIZE_32K */
	1040,  /* eBUFSIZE_64K */
	1480,  /* eBUFSIZE_128K */
	2090,  /* eBUFSIZE_256K */
};

static uint32_t getSyncCount(uint32_t traceBufSize )
{
	eSTM_ExportBufSize asyncIndex = eBUFSIZE_0;

	if (( traceBufSize > 0 ) && ( traceBufSize <= 4096 )) asyncIndex = eBUFSIZE_4K;
	else if (( traceBufSize > 4096 ) && ( traceBufSize <= 8192 )) asyncIndex = eBUFSIZE_8K;
	else if (( traceBufSize > 8192 ) && ( traceBufSize <= 16384 )) asyncIndex = eBUFSIZE_16K;
	else if (( traceBufSize > 16384 ) && ( traceBufSize <= 32768 )) asyncIndex = eBUFSIZE_32K;
	else if (( traceBufSize > 32768 ) && ( traceBufSize <= 65536 )) asyncIndex = eBUFSIZE_64K;
	else if (( traceBufSize > 65536 ) && ( traceBufSize <= 131072 )) asyncIndex = eBUFSIZE_128K;
	else if (( traceBufSize > 131072 ) && ( traceBufSize <= 262144 )) asyncIndex = eBUFSIZE_256K;

	return sync_count_table[asyncIndex];

}

#endif /* __linux */

