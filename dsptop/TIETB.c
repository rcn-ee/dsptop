/****************************************************************************
CToolsLib - ETB Library

Copyright (c) 2009-2014 Texas Instruments Inc. (www.ti.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/*! \file TIETB.c
    \version 1.7
*/
//#include <stdio.h>
//#include <stdlib.h>

#include "ETBInterface.h"
#include "ETBAddr.h"

#ifdef DMA_SUPPORT
#include "edma_dev-c66xx.h"

#if defined(C66AK2Hxx_CSSTM_ETB)

#define GET_GLOBAL_ADDR(addr) (uint32_t)(addr)

#else

#include "c6x.h"
#define GET_GLOBAL_ADDR(addr) \
    (uint32_t)(((uint32_t)(addr)) < (uint32_t)0x00900000 ? \
    ((uint32_t)(addr) | (uint32_t)((DNUM + 16) << 24)) : (uint32_t)(addr))

#endif

#endif

#ifdef __linux
extern void * cTools_memMap(uint32_t map_addr, uint32_t map_size);
extern void cTools_memUnMap(void * v_map_addr, uint32_t map_size);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////
//
// Private structs
//
////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(C66AK2Hxx) && defined(DMA_SUPPORT)

static const uint32_t keystone2_jtagId_table[] = {
		0x0b98102f, /* TCI6638K2K, TCI6636K2H, 66AK2Hxx */
		0x0b9a702f, /* TCI6630K2L */
		0x0b9a602f  /* 66AK2Exx, AM5K2Exx */
};

const uint32_t keystone2_jtagId_table_elements = sizeof(keystone2_jtagId_table)/sizeof(uint32_t);

/* The index of the JTAG ID table is used to select EDMA parameters in ETB_config_dma() */

static const k2_edma_config_table_t keystone2_edma_config_table [sizeof(keystone2_jtagId_table)/sizeof(uint32_t)][NUM_ETB_INSTANCES] = {
		/* TCI6638K2K, TCI6636K2H, 66AK2Hxx */
		{{3, EDMACC3_TETBHFULLINT0, EDMACC3_TETBFULLINT0}, /* DSP 0 */
		 {3, EDMACC3_TETBHFULLINT1, EDMACC3_TETBFULLINT1}, /* DSP 1 */
		 {3, EDMACC3_TETBHFULLINT2, EDMACC3_TETBFULLINT2}, /* DSP 2 */
		 {3, EDMACC3_TETBHFULLINT3, EDMACC3_TETBFULLINT3}, /* DSP 3 */
		 {2, EDMACC2_TETBHFULLINT4, EDMACC2_TETBFULLINT4}, /* DSP 4 */
		 {2, EDMACC2_TETBHFULLINT5, EDMACC2_TETBFULLINT5}, /* DSP 5 */
		 {2, EDMACC2_TETBHFULLINT6, EDMACC2_TETBFULLINT6}, /* DSP 6 */
		 {2, EDMACC2_TETBHFULLINT7, EDMACC2_TETBFULLINT7}, /* DSP 7 */
		 {4, EDMACC4_DBGTBR_DMAINT, (uint32_t)-1},         /* SYS TBR */
		 {4, EDMACC4_TETRISTBR_DMAINT, (uint32_t)-1}},     /* ARM TBR */

		/** TCI6630K2L */
		{{2, EDMACC2_TETBHFULLINT4, EDMACC2_TETBFULLINT4}, /* DSP 0 */
		 {2, EDMACC2_TETBHFULLINT5, EDMACC2_TETBFULLINT5}, /* DSP 1 */
		 {2, EDMACC2_TETBHFULLINT6, EDMACC2_TETBFULLINT6}, /* DSP 2 */
		 {2, EDMACC2_TETBHFULLINT7, EDMACC2_TETBFULLINT7}, /* DSP 3 */
		 {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
		 {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
		 {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
		 {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
		 {1, EDMACC1_DBGTBR_DMAINT, (uint32_t)-1},         /* SYS TBR */
		 {1, EDMACC1_TETRISTBR_DMAINT, (uint32_t)-1}},     /* ARM TBR */

		/* 66AK2Exx, AM5K2Exx */
		{{3, C66AK2E_EDMACC3_TETBHFULLINT0, C66AK2E_EDMACC3_TETBFULLINT0}, /* DSP 0 */
         {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
		 {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
	     {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
	     {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
	     {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
	     {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
	     {(uint32_t)-1, (uint32_t)-1, (uint32_t)-1},
		 {4, EDMACC4_DBGTBR_DMAINT, (uint32_t)-1},         /* SYS TBR */
		 {4, EDMACC4_TETRISTBR_DMAINT, (uint32_t)-1}},     /* ARM TBR */
};


#endif

#if defined(C6657) || defined(C66AK2Hxx)

/* DMA config structure used internally
   for C6657 and keystone2 devices
*/
typedef struct _DMAConfigInt
{

    uint32_t cc;           /* EDMA3 channel controller number. */

    uint16_t etbhalfChannel;   /* ETB half full channel for DMA draining (needs to
                                 be able to access the ETB).
                            */
    uint16_t etbfullChannel;   /* ETB full channel for DMA draining (needs to
                                 be able to access the ETB).
                            */
    uint16_t linkparam[3]; /* 3 additional parameter RAM entry numbers.
                            */
    uint32_t dbufAddress;  /* DMA Drain buffer address */
    uint32_t dbufWords;    /* DMA Drain buffer size in 32-bit words */
    eDMA_Mode mode;        /* DMA Drain buffer mode (only eDMA_Circular and
                                 eDMA_Stop_Buffer are valid values)
                            */
} DMAConfigInt;

#endif

struct _ETBHandle_t
{
    uint32_t ulContext;    /*!< ETB context handle*/
    uint8_t id;            /*!< ETB core user ID*/
    uint8_t dnum;          /*!< Detected CPU ID*/
    ETB_errorCallback pCallBack; /*!< ETB error callback*/

#if defined(C6657) || defined(C66AK2Hxx)

    DMAConfigInt *pDmaConfig; /*!< ETB DMA pointer, NULL when dma not used */

#else

    DMAConfig *pDmaConfig; /*!< ETB DMA pointer, NULL when dma not used */

#endif

    DMAStatus dmaStatus;   /*!< Copy of ETB DMA status, populated when
                                 ETB_flush_dma is called, used during ETB_read
                                 calls.
                            */
};

#ifdef __linux
static uint32_t virtual_ETB_BaseAddress[NUM_ETB_INSTANCES] = {0};
static uint32_t virtual_PSC_BaseAddress = 0;    
#endif

// Max timeout for ETB DMA transfers to complete
#define ETB_DMA_TIMEOUT 100000

/* ETB module access handle - virual for this library */
#ifndef __linux
#pragma DATA_SECTION(stHandle,"ETBLib_ExtMem");
#endif
static ETBHandle stHandle[NUM_ETB_INSTANCES];

/* Enumeration to define the options available to the internal flush function */
typedef enum _eETB_Options
{
    eETB_OPT_NONE = 0,    /*!< No options selected */
    eETB_STOP_FORMATTER   /*!< Stop the formatter once a flush has completed */
} eETB_OPTIONS;

/* Enumeration for ETB and TBR types */
typedef enum _ETB_Type
{
    TBR_TYPE  = 0,     /*!< TBR type */
    ETB_TYPE          /*!< ETB type */
}ETB_Type;

/* Internal ETB flush function used by API function for common purposes */
static eETB_Error flush(ETBHandle *pHandle, eETB_OPTIONS options);

/*! RETURN_ETB_CALLBACK
    ImplementaMacro to return API error.
*/
#define RETURN_ETB_CALLBACK(id,retValue) \
    if ( 0 != stHandle[id].pCallBack ) stHandle[id].pCallBack(retValue); return retValue

#ifndef __linux
// ETBLib symbols for CCS ETB Receiver (Trace capture and decoding)
// ETBLib symbols need to be placed in external memory (MSMC or DDR3)
// for STM driver to read from these symbols

#pragma DATA_SECTION(etbLib_buffer_start_addr,"ETBLib_ExtMem");
#pragma DATA_SECTION(etbLib_buffer_size,"ETBLib_ExtMem");
#pragma DATA_SECTION(etbLib_buffer_data_start,"ETBLib_ExtMem");
#endif
volatile uint32_t etbLib_buffer_start_addr[NUM_ETB_INSTANCES]; //start of ETB buffer
volatile uint32_t etbLib_buffer_size[NUM_ETB_INSTANCES]; //number of Bytes
volatile uint32_t etbLib_buffer_data_start[NUM_ETB_INSTANCES]; //address after circular buffer wrap point, where oldest data starts

#if SYSETB_PRESENT

#ifndef __linux
#pragma DATA_SECTION(etbLib_sys_etb_index,"ETBLib_ExtMem");
#endif
volatile uint32_t etbLib_sys_etb_index = SYS_ETB_ID; //symbol for SYS_ETB index

#endif

/*****************************************************************
 * Data that will be accessed by EDMA3 HW.
 * Application should link this data into appropriate memory so
 * that interference to the application is minimized
 *****************************************************************/
/*
 * If this data section is located in MSMC or DDR3 memory, it should
 *  be put in a non-cacheable region.
 */
#ifdef DMA_SUPPORT

#pragma DATA_SECTION(etbLib_bufferWrapped,"ETBLib_dmaData");

#if defined(C6670)

#pragma DATA_SECTION(etbLib_cpCicEventClearValue,"ETBLib_dmaData");
#pragma DATA_SECTION(etbLib_cpCicEventClearIndexReg,"ETBLib_dmaData");

static uint32_t etbLib_cpCicEventClearValue[2][NUM_ETB_INSTANCES][2] = {
{
    {CIC1_EVT_TETBHFULLINT0, CIC1_EVT_TETBFULLINT0},
    {CIC1_EVT_TETBHFULLINT1, CIC1_EVT_TETBFULLINT1},
    {CIC1_EVT_TETBHFULLINT2, CIC1_EVT_TETBFULLINT2},
    {CIC1_EVT_TETBHFULLINT3, CIC1_EVT_TETBFULLINT3},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {CIC1_EVT_TETBHFULLINT, CIC1_EVT_TETBFULLINT}
},
{
    {CIC2_EVT_TETBHFULLINT0, CIC2_EVT_TETBFULLINT0},
    {CIC2_EVT_TETBHFULLINT1, CIC2_EVT_TETBFULLINT1},
    {CIC2_EVT_TETBHFULLINT2, CIC2_EVT_TETBFULLINT2},
    {CIC2_EVT_TETBHFULLINT3, CIC2_EVT_TETBFULLINT3},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {CIC2_EVT_TETBHFULLINT, CIC2_EVT_TETBFULLINT}
}
};

static uint32_t etbLib_cpCicEventClearIndexReg[2] = {CIC1_STATUS_CLR_INDEX_REG, CIC2_STATUS_CLR_INDEX_REG};

#elif defined(C6678)

#pragma DATA_SECTION(etbLib_cpCicEventClearValue,"ETBLib_dmaData");
#pragma DATA_SECTION(etbLib_cpCicEventClearIndexReg,"ETBLib_dmaData");

static uint32_t etbLib_cpCicEventClearValue[2][NUM_ETB_INSTANCES][2] =
{
    {
        {CIC2_EVT_TETBHFULLINT0, CIC2_EVT_TETBFULLINT0},
        {CIC2_EVT_TETBHFULLINT1, CIC2_EVT_TETBFULLINT1},
        {CIC2_EVT_TETBHFULLINT2, CIC2_EVT_TETBFULLINT2},
        {CIC2_EVT_TETBHFULLINT3, CIC2_EVT_TETBFULLINT3},
        {CIC2_EVT_TETBHFULLINT4, CIC2_EVT_TETBFULLINT4},
        {CIC2_EVT_TETBHFULLINT5, CIC2_EVT_TETBFULLINT5},
        {CIC2_EVT_TETBHFULLINT6, CIC2_EVT_TETBFULLINT6},
        {CIC2_EVT_TETBHFULLINT7, CIC2_EVT_TETBFULLINT7},
        {CIC2_EVT_TETBHFULLINT, CIC2_EVT_TETBFULLINT}
    },
    {
        {CIC3_EVT_TETBHFULLINT0, CIC3_EVT_TETBFULLINT0},
        {CIC3_EVT_TETBHFULLINT1, CIC3_EVT_TETBFULLINT1},
        {CIC3_EVT_TETBHFULLINT2, CIC3_EVT_TETBFULLINT2},
        {CIC3_EVT_TETBHFULLINT3, CIC3_EVT_TETBFULLINT3},
        {CIC3_EVT_TETBHFULLINT4, CIC3_EVT_TETBFULLINT4},
        {CIC3_EVT_TETBHFULLINT5, CIC3_EVT_TETBFULLINT5},
        {CIC3_EVT_TETBHFULLINT6, CIC3_EVT_TETBFULLINT6},
        {CIC3_EVT_TETBHFULLINT7, CIC3_EVT_TETBFULLINT7},
        {CIC3_EVT_TETBHFULLINT, CIC3_EVT_TETBFULLINT}
    }
};

static uint32_t etbLib_cpCicEventClearIndexReg[2] = {CIC2_STATUS_CLR_INDEX_REG, CIC3_STATUS_CLR_INDEX_REG};

#endif

static uint32_t etbLib_bufferWrapped[NUM_ETB_INSTANCES][2];

#pragma DATA_SECTION(etb_disable,"ETBLib_dmaData");
static uint32_t etb_disable = 0x0;

#if defined(C66AK2Hxx)
//This buffer is used to temporary store trace samples, once the ETB-DMA buffer encounters the stop on full condition
//Required only for TBR implementations
#pragma DATA_SECTION(etb_stop_on_full_tempBuffer,"ETBLib_dmaData");
static uint8_t etb_stop_on_full_tempBuffer[ETB_BURST_SIZE];
#endif

#endif  //#ifdef DMA_SUPPORT

/**
* Normalize_ID(uint32_t id) - Normalize core/sys ETB ID/ARM ETB ID
*/
static uint32_t Normalize_ID(uint32_t id)
{
#if SYSETB_PRESENT
    if(id == SYS_ETB || id == SYS_ETB_ID)
    {
        id = SYS_ETB_ID;
    }
#endif
#if ARMETB_PRESENT
    if(id == ARM_ETB || id == ARM_ETB_ID)
    {
        id = ARM_ETB_ID;
    }
#endif

    return id;
}

/**
* Handle_index(uint32_t id) - Get handle id for the core/sys ETB
*/
static uint32_t Handle_index(uint32_t id)
{
#if (NUM_ETB_INSTANCES == 1) && SYSETB_PRESENT  /* To avoid a gcc compiler warning */
return SYS_ETB_ID;
#else
#if SYSETB_PRESENT
    if(id == SYS_ETB || id == SYS_ETB_ID)
    {
        id = SYS_ETB_ID;
    }
#endif

#if ARMETB_PRESENT
    if(id == ARM_ETB || id == ARM_ETB_ID)
    {
        id = ARM_ETB_ID;
    }
#endif
#endif
    return id;
}

/**
* Check whether the coreID passed is associated with a TI-ETB or TBR implementation
* returns TBR_TYPE = 0 - for TBR implementation and ETB_TYPE = 1 - for TI-ETB implementation
*/
//Please Note: Do not make this function static to the ETBLib
ETB_Type check_etb_type(uint32_t id)
{
	uint32_t dev_type = ((*((volatile uint32_t*)ETB_PERIPH_ID1(id)) & ETB_PERIPH_ID1_MASK) << 8)
			            | (*((volatile uint32_t*)ETB_PERIPH_ID0(id)) & ETB_PERIPH_ID0_MASK);

	if (dev_type == TBR_DEVICE_TYPE)
	{
		return(TBR_TYPE);
	}

	//By default always return TI-ETB type
	return(ETB_TYPE);
}

#if defined(TCI6484) || defined(C66x) || defined(C66AK2Hxx)
#ifndef __linux
/**
* Workaround_portownership() - Work around to setup correct debug port ownership
*/
static void Workaround_portownership()
{
    uint32_t status=0;
    status = *(volatile uint32_t*)0x1bc013c;
    status= status | 01;
    *(volatile uint32_t*)0x1bc013c = status;
}
#endif
#endif

#if !defined(_OMAP54xx) && !defined(_DRA7xx) && !defined(TDA3x_C66x)

#if !defined(TCI6484) && !defined(TCI6486)
//Keystone specific PSC setup
//Please Note: Do not make this function static to the ETBLib
eETB_Error Keystone_Program_Power_Sleep()
{
    uint32_t status=0;
    uint32_t ptcmdValue = 0;
    uint32_t retry = 1000;
	uint32_t etb_power_state;
	uint32_t debugss_cptracers_enable;
	uint32_t etb_enable;

	/* If etb powered up and both etb and debugss enabled then exit */
	etb_power_state = *((volatile uint32_t*)PSC_PDSTAT(1));

	debugss_cptracers_enable = *((volatile uint32_t*)PSC_MDSTAT(5));

	etb_enable = *((volatile uint32_t*)PSC_MDSTAT(6));

	if ((etb_power_state & 1) && (debugss_cptracers_enable & 3) && (etb_enable & 3))
	{
		return eETB_Success;
	}

    /* Enable TETB power domain*/

    /* Power domain Go transition command for TETB power domain */
    ptcmdValue = 0x2;

    /* Power up DebugSS */
    *((volatile uint32_t*)PSC_PDCTL(1)) = 0x1;

    // Issue GO for the DebugSS power domain
    *((volatile uint32_t*)PSC_PTCMD) = ptcmdValue;

    //Wait for the domain transition to complete
    do
    {
       status = *(volatile uint32_t*)PSC_PTSTAT;
       retry--;
    }  while( ( ( status & ptcmdValue ) != 0 ) && ( retry != 0 ) );
    if ( retry == 0 ) return eETB_Error_Psc_Enabling;

    /* Enable DEBUGSS subsystem */
    *((volatile uint32_t*)PSC_MDCTL(5)) = 0x3;
    /* Enable Per-core TETB and system TETB subsystem */
    *((volatile uint32_t*)PSC_MDCTL(6)) = 0x3;

    /* Power domain Go transition command for enabled power domain */
    *((volatile uint32_t*)PSC_PTCMD) = ptcmdValue;

    //Wait for the domain transition to complete
    retry = 1000;
    do
    {
       status = *(volatile uint32_t*)PSC_PTSTAT;
       retry--;
    }  while( ( ( status & ptcmdValue ) != 0 ) && ( retry != 0 ) );
    if ( retry == 0 ) return eETB_Error_Psc_Enabling;

   /* Check that the modules have been properly enabled */
    if ((*((volatile uint32_t*)PSC_MDSTAT(5)) & 0x3F) != 0x3 ||
        (*((volatile uint32_t*)PSC_MDSTAT(6)) & 0x3F) != 0x3)
        return eETB_Error_Psc_Enabling;

#if defined(C66x) || defined(C66AK2Hxx)
/*Can't access port ownership location from ARM */
#ifndef __linux
    Workaround_portownership();
#endif
#endif

    return eETB_Success;
}
#endif

#if defined(TCI6484)
//TCI6484 specific PSC setup
static eETB_Error TCI6484_Program_Power_Sleep()
{
    uint32_t status=0;
    uint32_t ptcmdValue = 0;
    uint32_t retry = 1000;

    /* Enable DEBUGSS */
    *((volatile uint32_t*)PSC_MDCTL(6)) = 0x3;

    /* Power domain Go transition command */
    ptcmdValue = 0x1;

    /* Power domain Go transition command for enabled power domain */
    *((volatile uint32_t*)PSC_PTCMD) = ptcmdValue;

    //Wait for the domain transition to complete
    retry = 1000;
    do
    {
       status = *(volatile uint32_t*)PSC_PTSTAT;
       retry--;
    }  while( ( ( status & ptcmdValue ) != 0 ) && ( retry != 0 ) );
    if ( retry == 0 ) return eETB_Error_Psc_Enabling;

    if ((*((volatile uint32_t*)PSC_MDSTAT(6)) & 0x3F) != 0x3)
        return eETB_Error_Psc_Enabling;

    Workaround_portownership();

    return eETB_Success;
}
#endif

/**
* Program_Power_Sleep() - Work around to setup correct debug port ownership
*/
eETB_Error Program_Power_Sleep()
{
    /* Program Power and Sleep Controls */
#if defined(TCI6484)

	//TCI6484 specific PSC setup
	return(TCI6484_Program_Power_Sleep());

#elif defined(C66x) || defined(C66AK2Hxx)

	//Keystone specific PSC setup
	return(Keystone_Program_Power_Sleep());

#else

	return(eETB_Success);

#endif
}

#endif //#if !defined(_OMAP54xx) && !defined(_DRA7xx) && !defined(TDA3x_C66x)

#if defined(C6657) || defined(C66AK2Hxx)
#ifdef DMA_SUPPORT
static uint8_t get_edma_buffer_info (ETBHandle* pHandle, DMAStatus *pStatus, uint32_t paramAddress, int32_t remWords)
{
    int32_t remBytes;

    remBytes = (remWords * 4);

    /* Check for buffer wrap */
    if(etbLib_bufferWrapped[pHandle->id][0])
    {
        pStatus->availableWords    = pHandle->dmaStatus.dbufWords;
        pStatus->startAddr = PARAM_DST_REG(paramAddress) + remBytes;
        pStatus->isWrapped = 1;
    }
    else
    {
        pStatus->availableWords = (PARAM_DST_REG(paramAddress) -
                            pHandle->pDmaConfig->dbufAddress) / 4;
        pStatus->availableWords += remWords;
        pStatus->startAddr = pHandle->pDmaConfig->dbufAddress;
        pStatus->isWrapped = 0;
    }

    /* Copy the DMA status into the handle for later management. The status
     *  value for the number of words may be different than what was in the
     *  configuration structure. This value is set during configuration.
     */
    pStatus->dbufAddress = pHandle->pDmaConfig->dbufAddress;
    pStatus->dbufWords   = pHandle->dmaStatus.dbufWords;
    pStatus->flushRequired = pHandle->dmaStatus.flushRequired;
    pHandle->dmaStatus   = *pStatus;

    /* If the mode has been set to non-circular and the buffer wrapped flag
     *  is set, then the memory buffer if full, otherwise continue the
     *  configuration for the final DMA.
     */
    if((pHandle->pDmaConfig->mode == eDMA_Stop_Buffer) &&
        (etbLib_bufferWrapped[pHandle->id][0]))
    {
        /* If the buffer wrapped, the startAddr value is incorrect from
         *  above, set back to beginning of the buffer.
         */
        pStatus->startAddr = pHandle->pDmaConfig->dbufAddress;
        pHandle->dmaStatus.startAddr = pHandle->pDmaConfig->dbufAddress;
        return(1);
    }

    // Update the 3 symbols which are required for CCS ETB receiver
    etbLib_buffer_start_addr[pHandle->id] = pHandle->pDmaConfig->dbufAddress; //CCS ETB receiver will always get a linearized buffer for the non-EDMA ETB drain case
    etbLib_buffer_size[pHandle->id] = pStatus->availableWords * 4; //Number of bytes available
    etbLib_buffer_data_start[pHandle->id] = pStatus->startAddr; //circular buffer wrap point

    return(0);
}

#endif //#ifdef DMA_SUPPORT
#endif //#if defined(C6657) || defined(C66AK2Hxx)

/**
* ETB_open - open ETB programming module interface
*/

eETB_Error  ETB_open(ETB_errorCallback pErrCallBack, eETB_Mode mode, uint8_t coreID, ETBHandle** ppHandle, uint32_t* pETBSizeInWords)
{
    uint32_t status, value;

    coreID = Normalize_ID(coreID);
   
#if SYSETB_PRESENT 
    // populate SYS_ETB index symbol
    if(coreID == SYS_ETB_ID)
    {
        etbLib_sys_etb_index = SYS_ETB_ID;
    }
    
#endif

#ifndef SHARED_ETB_DEVICE
    if(Handle_index(coreID) >= NUM_ETB_INSTANCES)
    {
        return eETB_Error_Bad_Param;
    }
#endif

    if ((ppHandle == 0 ) || ( pETBSizeInWords == 0) )
    {
        return eETB_Error_Bad_Param;
    }

#ifdef __linux
    virtual_ETB_BaseAddress[coreID] = (uint32_t)cTools_memMap(ETB_BaseAddress(coreID), SIZEOF_ETB_SPACE);
#ifndef NO_PSC
    if (virtual_PSC_BaseAddress == 0)
    {
        virtual_PSC_BaseAddress = (uint32_t)cTools_memMap(PSC_BaseAddress, SIZEOF_PSC_SPACE);
    }
#endif
#endif

#if !defined(_OMAP54xx) && !defined(_DRA7xx) && !defined(TDA3x_C66x)
#if ARMETB_PRESENT
    if(Handle_index(coreID) != ARM_ETB_ID)
#endif
    {
#ifndef NO_PSC
    	eETB_Error retVal = eETB_Success;
    	retVal = Program_Power_Sleep();
	    if (retVal != eETB_Success)
            return retVal;
#endif
    }
#endif

#ifdef TDA3x_C66x
    //TODO - In the TDA3x case the C66x still needs the workaround (I think?) but for this release
    // Program_Power_Sleep() is not being called because we don't have documentation on how the
    // PSC works for TDA3x.
    Workaround_portownership();
#endif

    if(check_etb_type(coreID) == ETB_TYPE)
    {
        
        // Reset the ETB
        *((volatile uint32_t*)ETB_TI_CTL(coreID)) = *((volatile uint32_t*)ETB_TI_CTL(coreID)) | 0x4;

        /* Unlock ETB in order to enable accesses to any ETB registers below. */
        *((volatile uint32_t*)ETB_LOCK(coreID)) = ETB_UNLOCK_VAL;

        /* Size of the ETB. ETB_RDP contains number of 32 bit wide words. */
        *pETBSizeInWords = (*((volatile uint32_t*)ETB_RDP(coreID)));

        /* Setup ETB mode */
        if ((eETB_TI_Mode == mode) || (eETB_TI_Mode_AND_Stop_Buffer == mode))
            *((volatile uint32_t*)ETB_TI_CTL(coreID)) = TI_ETB_TI_MODE;

        status = *((volatile uint32_t*)ETB_TI_CTL(coreID));
        if((eETB_Stop_Buffer != mode) && (eETB_TI_Mode_AND_Stop_Buffer != mode))
        {
            /* Set TI ETB for circular mode - clear the bit. */
            *((volatile uint32_t*)ETB_TI_CTL(coreID)) = status & ~(TI_ETB_CIRCULARMODE_BIT);
        }
        else
        {
            /* Set TI ETB as stop buffer full mode - set the bit. */
            *((volatile uint32_t*)ETB_TI_CTL(coreID)) = status | TI_ETB_CIRCULARMODE_BIT;
        }
    }
    else //TBR implementation
    {
        // Reset the ETB
    	//Clear ETB_CTL register before issuing TBR soft reset
    	*((volatile uint32_t*)ETB_CTL(coreID)) = 0x0;
        *((volatile uint32_t*)ETB_CTL(coreID)) = *((volatile uint32_t*)ETB_CTL(coreID)) | 0x4;

        //Read claim tag value and check if TBR is not already claimed
        // Assumption here is that the debugger will set the claim bit to get ownership of the TBR
        status = *((volatile uint32_t*)TBR_CLAIMCLR(coreID));

        if(status != 0)
        {
            return eETB_Error_Cannot_Own;
        }

        /* Unlock to enable accesses */
        status = (*((volatile uint32_t*)ETB_LOCK_STATUS(coreID)));
        if (status & LOCK_STATUS_IMP_BIT)
        {
            /* If this bit is set, it device access is locked, we need to unlock the device*/
            if (status & LOCK_STATUS_STAT_BIT)
            {
                /* Unlock ETB in order to enable accesses to any ETB registers below. */
                *((volatile uint32_t*)ETB_LOCK(coreID)) = ETB_UNLOCK_VAL;
            }
        }

        /* Size of the ETB. ETB_RDP contains number of 32 bit wide words. */
        value = (*((volatile uint32_t*)ETB_RDP(coreID)));
        *pETBSizeInWords = (1 << (value-1)) << 10;

        /* Setup TBR mode */
        value = (*((volatile uint32_t*)ETB_FFCR(coreID)));

        /* Setup TBR in buffer mode */
        *((volatile uint32_t*)ETB_CTL(coreID)) &= TBR_BUFFER_MODE;

        if((eETB_Stop_Buffer == mode) || (eETB_TI_Mode_AND_Stop_Buffer == mode))
        {
            //Configure stop on Full mode
            (*((volatile uint32_t*)ETB_FFCR(coreID))) = value | TBR_STP_FULL;
        }
        else if((eETB_Circular == mode) || (eETB_TI_Mode == mode))
        {
            //Configure circular mode
            (*((volatile uint32_t*)ETB_FFCR(coreID))) = value & (~(uint32_t)TBR_STP_FULL);
        }
    }

    /* Set ETB context in the handle*/
    stHandle[Handle_index(coreID)].ulContext = ETB_UNLOCK_VAL;
    stHandle[Handle_index(coreID)].id = coreID;
    stHandle[Handle_index(coreID)].dnum = coreID;
    stHandle[Handle_index(coreID)].pCallBack = pErrCallBack;
    stHandle[Handle_index(coreID)].pDmaConfig = 0;
    stHandle[Handle_index(coreID)].dmaStatus.startAddr   = 0;
    stHandle[Handle_index(coreID)].dmaStatus.availableWords      = 0;
    stHandle[Handle_index(coreID)].dmaStatus.isWrapped   = 0;
    stHandle[Handle_index(coreID)].dmaStatus.dbufAddress = 0;
    stHandle[Handle_index(coreID)].dmaStatus.dbufWords   = 0;
    stHandle[Handle_index(coreID)].dmaStatus.flushRequired = 1;

    *ppHandle = &stHandle[Handle_index(coreID)];
    return eETB_Success;
}

eETB_Error  ETB_gethandle(uint8_t coreID, ETBHandle** ppHandle)
{
	/* If context set for coreID then return handle, else return eETB_Error_Bad_Param */
	if (stHandle[Handle_index(coreID)].ulContext == ETB_UNLOCK_VAL)
	{
	    *ppHandle = &stHandle[Handle_index(coreID)];
	    return eETB_Success;

	}
	else
	{
		return eETB_Error_Program;
	}
}

/**
* ETB_enable- Enable ETB to capture trace data
*/
eETB_Error  ETB_enable(ETBHandle* pHandle, uint32_t triggerCount)
{
    uint32_t etbControl;
    uint32_t waitCount = 1000;

    if(pHandle == 0)
    {
    	return eETB_Error_Bad_Param;
    }

    if(pHandle->ulContext != ETB_UNLOCK_VAL)
    {
//        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Bad_Param);
    }

    /* Make sure that tracing is disabled or the write to the following
     *  registers will not actually get written.
     */
    *((volatile uint32_t*)ETB_CTL(pHandle->id)) = *((volatile uint32_t*)ETB_CTL(pHandle->id)) & (~ETB_ENABLE);/* DISABLE TraceCaptEn */

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        /* ETB FIFO reset by writing 0 to ETB RAM Write Pointer Register. */
        *((volatile uint32_t*)ETB_RWP(pHandle->id)) = 0;

        /* Initialize RDP. */
        *((volatile uint32_t*)ETB_RRP(pHandle->id)) = 0;
    
        /* Clear all interrupts before enabling trace, just to make sure nothing pending*/
        *((volatile uint32_t*)ETB_ICST(pHandle->id)) = (TI_ETB_IRST_FULL |
                                                        TI_ETB_IRST_HALF_FULL |
                                                        TI_ETB_IRST_OVERFLOW |
                                                        TI_ETB_IRST_UNDERFLOW);
    }
    else
    {
        /* Clear all interrupts before enabling trace, just to make sure nothing pending*/
        *((volatile uint32_t*)TBR_IRQSTATUS(pHandle->id)) = (TBR_IRST_DAV |
                                                             TBR_IRST_AQCMP);
    }

    /* Disable formatting and put ETB formatter into bypass mode. */
    /* Clear all control bits except STP_FULL */
    *((volatile uint32_t*)ETB_FFCR(pHandle->id)) &= TBR_STP_FULL; /* For TI-ETB: EnFCont=0, EnFTC=0 */

    if(check_etb_type(pHandle->id) == TBR_TYPE)
    {
        //Set ID period to 8. Artificial ID will be added, if there is no ID change for 8 TWP frames
        *((volatile uint32_t*)TBR_IDPERIOD(pHandle->id)) = TBR_TWP_IDPERIOD;

        *((volatile uint32_t*)TBR_SEQCNTL(pHandle->id)) = TBR_TWP_SEQATBID | TBR_TWP_SEQPERIOD;

        //Always enable TBR TWP formatter
        *((volatile uint32_t*)ETB_FFCR(pHandle->id)) = *((volatile uint32_t*)ETB_FFCR(pHandle->id)) | TBR_TWP_ENABLE;
    }

    /* Setup Trigger counter. */
    *((volatile uint32_t*)ETB_TRIG(pHandle->id)) = triggerCount;

#ifdef DMA_SUPPORT
    /* If the DMA has been enabled, indicated by dma pointer non-null, enable
     *  the ETB half-full and full interrupts.
     */
    if(pHandle->pDmaConfig != 0)
    {
        if(check_etb_type(pHandle->id) == ETB_TYPE)
        {
            *((volatile uint32_t*)ETB_IER(pHandle->id))  = (TI_ETB_IRST_FULL |
                                                            TI_ETB_IRST_HALF_FULL);
        }
    }
#endif

    /* Enable ETB data capture by writing ETB Control Register. */
    *((volatile uint32_t*)ETB_CTL(pHandle->id)) = *((volatile uint32_t*)ETB_CTL(pHandle->id)) | ETB_ENABLE; /* TraceCaptEn =1 */

    /* Put some delays in here - make sure we can read back. */
    do
    {
        etbControl = *((volatile uint32_t*)ETB_CTL(pHandle->id));
        waitCount--;
    } while (((etbControl & 0x1) != ETB_ENABLE) && (waitCount > 0));

    if(waitCount ==0)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    return eETB_Success;
}

/**
* ETB_disable - Disable ETB to stop capturing trace data
*/
eETB_Error  ETB_disable(ETBHandle* pHandle)
{
    eETB_Error  ret = eETB_Success;

    if(pHandle == 0)
    {
    	return eETB_Error_Bad_Param;
    }

    if(pHandle->ulContext != ETB_UNLOCK_VAL)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    /* Flush ETB, wait until after disabling data capture to check status.
     * This function will not return until the flush has completed.
     */
     // Perform Flush + stop formatter only for non-EDMA examples. For EDMA examples 
     // the Flush + stop formatter is already done using the ETB_flush() API  
    if(pHandle->pDmaConfig == 0)
    {
        ret = flush(pHandle, eETB_STOP_FORMATTER);
    }

    /* Disable ETB data capture by writing ETB Control Register. */
    *((volatile uint32_t*)ETB_CTL(pHandle->id)) = *((volatile uint32_t*)ETB_CTL(pHandle->id)) & (~ETB_ENABLE); /* TraceCaptEn =0 */

    if(ret != eETB_Success)
    {
        RETURN_ETB_CALLBACK(pHandle->id, ret);
    }

    // Wait for ETB Acquisition Complete
    /* ==> The code to check the ETB_STS register for ETB_STS_ACQCOMP bit
     *      is not necessary if a manual flush has occurred and
     *      completed. For the TBR case, the disable trace capture will
     *      cause a flush, polling this bit would serve a purpose.
     */

#ifdef DMA_SUPPORT
    /* If the DMA has been enabled, indicated by dma pointer non-null, disable
     *  the ETB half-full and full interrupts.
     */
    if(pHandle->pDmaConfig != 0)
    {
        if(check_etb_type(pHandle->id) == ETB_TYPE)
        {
            *((volatile uint32_t*)ETB_IECST(pHandle->id))  = (TI_ETB_IRST_FULL |
                                                          TI_ETB_IRST_HALF_FULL);
        }
        else
        {
            *((volatile uint32_t*)TBR_IRQENABLE_CLR(pHandle->id)) = TBR_IRST_DAV;
        }
    }
#endif
    return ret;
}


/**
* ETB_status- Get ETB status
*
    ETB status register .
    STS:
    RAMFull=1   [0], RAMEmpty=0     [0]
    Triggered=1 [1], NotTriggered=0 [1]
    AccqComp=1  [2], NotAccqComp=0  [2]
    FtEmpty=1   [3], NotFtEmpty=0   [3]
    x --> reserved
    v --> read value
    31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    ________________________________________________________________________________________________
    |x | x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| x| v| v| v| v|
    |__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|

*/
eETB_Error  ETB_status(ETBHandle* pHandle, ETBStatus* status)
{
    uint32_t etbStatus, etbControl=0, etb_size;

    if(pHandle == 0)
    {
    	return eETB_Error_Bad_Param;
    }

    if(pHandle->ulContext != ETB_UNLOCK_VAL)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    if(status == 0 )
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Bad_Param);
    }

    //Get ETB Trace capture enable status
    status->ETB_TraceCaptureEn = *((volatile uint32_t*)ETB_CTL(pHandle->id));

#ifdef DMA_SUPPORT
    
    if(pHandle->pDmaConfig != 0)
    {
        uint32_t paramAddress;
        uint16_t paramIdx;
        
#if defined(C6657) || defined(C66AK2Hxx)

        paramIdx = (EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                                     pHandle->pDmaConfig->etbhalfChannel) >> 5);

#elif defined(C6670) || defined(C6678)

        paramIdx = (EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                                     pHandle->pDmaConfig->etbChannel) >> 5);
#endif

        paramAddress = EDMA_TPCC_PARAM_BASE_ADDR(pHandle->pDmaConfig->cc) +
                                                    (0x20 * paramIdx);

        /* Check for buffer wrap */
        if(etbLib_bufferWrapped[pHandle->id][0])
        {
            status->availableWords = pHandle->dmaStatus.dbufWords;
            status->isWrapped = 1;
            status->canRead   = 1;
        }
        else
        {
            status->availableWords = (PARAM_DST_REG(paramAddress) -
                                pHandle->pDmaConfig->dbufAddress) / 4;
            status->isWrapped = 0;
            status->canRead   = 1;
        }
    }
    else
#endif
    {
        status->canRead=0;
        status->availableWords=0;
        status->isWrapped=0;
        status->overflow = 0;
        etbStatus = *((volatile uint32_t*)ETB_STS(pHandle->id));

        if (etbStatus & TI_ETB_IRST_OVERFLOW) 
        {
            status->overflow = 1;
        }

        if(check_etb_type(pHandle->id) == ETB_TYPE)
        {
            if ((TI_ETB_TI_MODE & *((volatile uint32_t*)ETB_TI_CTL(pHandle->id))) ==
                    TI_ETB_TI_MODE)
            {
                /* In this case we must take into account the number of words already read */
                status->canRead = 1;
            }
            else
            {

                etbControl = *((volatile uint32_t*)ETB_CTL(pHandle->id));

                if(etbControl == ETB_ENABLE)
                {
                    return eETB_Success;
                }

                if((etbStatus & ETB_STS_ACQCOMP) == ETB_STS_ACQCOMP)
                {
                    status->canRead = 1;
                }
            }

            // get size of the ETB buffer in words
            etb_size = (*((volatile uint32_t*)ETB_RDP(pHandle->id)));
        }
        else
        {
            etbControl = *((volatile uint32_t*)ETB_CTL(pHandle->id));

            if((etbStatus & ETB_STS_ACQCOMP) == ETB_STS_ACQCOMP)
            {
                status->canRead = 1;
            }

            // get size of the TBR buffer in words
            etb_size = (*((volatile uint32_t*)ETB_RDP(pHandle->id)));
            etb_size = (1 << (etb_size-1)) << 10;
        }

        if ((etbStatus  & ETB_STS_FULL) == ETB_STS_FULL)
        {
            status->isWrapped = 1;
            status->availableWords = etb_size;
        }
        else
            status->availableWords = *((volatile uint32_t*)ETB_RWP(pHandle->id));
    }
    return eETB_Success;
}


/**
* ETB_read- Read ETB data
*
*  Reference Diagram for DMA drain buffer parameters:
*
*    Wrapped buffer:                          Non-Wrapped buffer:
*    _______________                          _______________
*   |_______________| -> dbufStartAddr       |_______________| -> dbufStartAddr
*   |               |                        |               |  && dataStartAddr
*   |               |                        |               |
*   //              //                       //              //
*   |_______________|                        //              //
*   |_______________| -> dataEndAddr         |_______________|
*   |_______________| -> dataStartAddr       |_______________| -> dataEndAddr
*   |               |                        |  \\\\\\\\\\\\ |
*   |               |                        |  //////////// |  No data
*   //              //                       // \\\\\\\\\\\\ //
*   |_______________|                        |  //////////// |
*   |_______________| -> dbufEndAddr         |_______________| -> dbufEndAddr
*
*/
eETB_Error ETB_read(ETBHandle* pHandle,    uint32_t *pBuffer,
                    uint32_t bufferLength, uint32_t startWord,
                    uint32_t readLength, uint32_t* pRetLength)
{
    ETBStatus status;
    uint32_t idx, startAddr, depth, value;

    if(pHandle == 0)
    {
    	return eETB_Error_Bad_Param;
    }

    if(pHandle->ulContext != ETB_UNLOCK_VAL)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
    /* Check if we have proper buffer pointer */
    if(pBuffer == 0)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Bad_Param);
    }

    *pRetLength =0;

#ifdef DMA_SUPPORT
    /* If the DMA has been configured to empty the ETB, then read from the memory
     *  buffer instead of the ETB hardware registers.
     */
    if(pHandle->pDmaConfig != 0)
    {
        uint32_t *pDmaBuffer;
        uint32_t dataStartAddr; // Start of data address in case of wrap
        uint32_t dataEndAddr; // End of data address
        uint32_t wordCnt;     // Number of words that will be copied
        int32_t  remWords;    // Remaining words to read from drain buffer
        uint32_t dbufEndAddr; // End of buffer address
        uint32_t dbufStartAddr = pHandle->dmaStatus.dbufAddress;

        /* Set/calculate address values */
        dataStartAddr = pHandle->dmaStatus.startAddr;
        dbufEndAddr = pHandle->dmaStatus.dbufAddress + (pHandle->dmaStatus.dbufWords*4) - 1;

        /* If the data starting address is not equal to the buffer starting address,
         *  then the buffer has wrapped, the ending data address will be
         *  the previous word's address. Otherwise, it will be the starting address
         *  plus the number of words written into the drain buffer.
         */
        if(dataStartAddr > dbufStartAddr)
        {
            dataEndAddr = dataStartAddr - sizeof(uint32_t);
        }
        else
        {
            dataEndAddr = dataStartAddr + ((pHandle->dmaStatus.availableWords-1) * 4);
        }
        /* The startWord variable is passed into this function as an offset for
         *  this read request. The value is translated into bytes from words to
         *  get the correct address value to start reading words from the drain
         *  buffer.
         */
        startAddr   = dataStartAddr + (startWord * 4);

        /* Check to see if the starting address is past the end of the buffer */
    //    if(startAddr > dbufEndAddr)
     //   {
       //     /* Subtract 1 from the difference to account for 0-based counting */
        //    startAddr = ((startAddr - dbufEndAddr) + dbufStartAddr) - 1;
       // }

        /* Initially set the total word count to the specified buffer length,
         *  then check if the requested read length is less, then finally if
         *  the remaining words left in the drain buffer is less.
         */
        wordCnt = bufferLength;
        if(readLength < wordCnt)
        {
            wordCnt = readLength;
        }

        /* The following calculation is using the remaining words variable to calculate
         *  byte address values. The conversion from bytes to words will occur after the
         *  calculations have been completed.
         */
        remWords = dataEndAddr - startAddr;
        if(remWords < 0)
        {
            if(pHandle->dmaStatus.isWrapped)
            {
                /* 1 is added back to the end address to use the size vs. address */
                remWords = ((dbufEndAddr+1) - startAddr) + (dataEndAddr - dbufStartAddr);
            }
            else
            {
                remWords = 0;
            }
        }
        /* Convert from bytes to words */
        if(remWords != 0)
        {
            remWords /= 4;
            remWords += 1; /* 1 added for 0-based counting */
        }

        if(remWords < wordCnt)
        {
            wordCnt = remWords;
        }

        /* Loop through and copy the data from the ETB buffer into the provided
         *  buffer.
         */
        pDmaBuffer = (uint32_t *)startAddr;
        for(idx = 0; idx < wordCnt; idx++)
        {
            /* Check for startAddr past the total words in the buffer and set
             *  the pointer/address values to the buffer's starting address.
             */
            if(startAddr > dbufEndAddr)
            {
                pDmaBuffer = (uint32_t *)(pHandle->dmaStatus.dbufAddress + (startAddr - (dbufEndAddr+1)));
                startAddr  = pHandle->dmaStatus.dbufAddress;
            }
            pBuffer[idx] = *pDmaBuffer++;

            (*pRetLength)++;

            startAddr += 4; /* Increment by full-word */
        }
    }
    else
#endif //#ifdef DMA_SUPPORT
    {
        /* Get the status of the ETB before we can start reading. */
        if(eETB_Success == ETB_status(pHandle, &status))
        {
            if(status.canRead == 0)
            {
                RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Cannot_Read);
            }
            
            // Update the 3 symbols which are required for CCS ETB receiver
            etbLib_buffer_start_addr[pHandle->id] = (uint32_t)pBuffer; //CCS ETB receiver will always get a linearized buffer for the non-EDMA ETB drain case
            etbLib_buffer_size[pHandle->id] = status.availableWords * 4; //Number of bytes available 
            etbLib_buffer_data_start[pHandle->id] = etbLib_buffer_start_addr[pHandle->id]; //circular buffer wrap point, if equal to the start address, no unwrapping of the buffer is required 

            if(check_etb_type(pHandle->id) == ETB_TYPE)
            {
                /* ETB depth */
                depth = *((volatile uint32_t*)ETB_RDP(pHandle->id));
            }
            else
            {
                /* ETB depth */
                value = (*((volatile uint32_t*)ETB_RDP(pHandle->id)));
                depth = (1 << (value-1)) << 10;
            }

            /* Check if the buffer is wrapped or not; set read pointers
             *  accordingly.
             */
            if(status.isWrapped == 1)
            {
                startAddr =
                    *((volatile uint32_t*)ETB_RWP(pHandle->id)) + startWord;
            }
            else
            {
                startAddr = 0x0 + startWord;
            }

            /* Adjust the read size for the available user buffer and requested
             *  data.
             */
            if(bufferLength < status.availableWords)
                status.availableWords = bufferLength;

            if(readLength < status.availableWords)
                status.availableWords = readLength;

            /* Adjust to accomodate the start word. */
            if(startWord < status.availableWords)
                status.availableWords = status.availableWords - startWord;
            else
                status.availableWords =0;

            if(check_etb_type(pHandle->id) == ETB_TYPE)
            {
                // Not valid to write the RRP if TIETB has not wrapped
                if(status.isWrapped == 1)
                {
                    /* Initialize the ETB RAM read pointer register with startAddr*/
                    *((volatile uint32_t*)ETB_RRP(pHandle->id)) = startAddr;

                    /* Clear the overflow flag*/
                    *((volatile uint32_t*)ETB_ICST(pHandle->id)) = TI_ETB_IRST_OVERFLOW;
                }
            }
            else
            {
                /* Initialize the ETB RAM read pointer register with startAddr*/
                *((volatile uint32_t*)ETB_RRP(pHandle->id)) = startAddr;
            }

            /* Now read trace data out of ETB */
            for (idx = 0; idx < status.availableWords; idx++)
            {
                /* Read the ETB RAM read data register to retrieve trace data.
                 *  This would cause the read pointer register value to
                 *  auto-increment.
                 */
                pBuffer[idx] = *((volatile uint32_t*)ETB_RRD(pHandle->id));

                (*pRetLength)++;

                startAddr++;

                if(startAddr == depth)
                {
                    /*Now wrap from begining */
                    startAddr = 0x0;
                    *((volatile uint32_t*)ETB_RRP(pHandle->id)) = startAddr;
                }
            }
        }
    }

    return eETB_Success;
}


/**
* ETB_close- Close ETB programming module interface
*/
eETB_Error  ETB_close(ETBHandle* pHandle)
{
    if(pHandle == 0)
    {
    	return eETB_Error_Bad_Param;
    }

    if(pHandle->ulContext != ETB_UNLOCK_VAL)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    if(pHandle->ulContext == ETB_UNLOCK_VAL)
    {
        pHandle->ulContext = 0;
    }

#ifdef __linux
    {
        cTools_memUnMap((void *)virtual_ETB_BaseAddress[pHandle->id], SIZEOF_ETB_SPACE);
        virtual_ETB_BaseAddress[pHandle->id] = 0;

#ifndef NO_PSC
        int i;
        /* Only close the PSC if no other instances of ETBs are open */
        for (i = 0; i < NUM_ETB_INSTANCES; i++) {
            if (virtual_ETB_BaseAddress[pHandle->id] != 0) break;
        }
        if ( i == NUM_ETB_INSTANCES) 
        { 
            cTools_memUnMap((void *)virtual_PSC_BaseAddress, SIZEOF_PSC_SPACE);
            virtual_PSC_BaseAddress = 0;
        }
#endif
    }
#endif

    return eETB_Success;
}

/**
* ETB_flush - Flush the ETB.
*/
eETB_Error  ETB_flush(ETBHandle* pHandle)
{
    eETB_Error  ret;

    if(pHandle == 0)
    {
    	return eETB_Error_Bad_Param;
    }

    if(pHandle->ulContext != ETB_UNLOCK_VAL)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

#ifdef DMA_SUPPORT
    // Stop the formatter + Flush to the ETB, if EDMA mode is enabled or its a STM-ETB
    if((pHandle->pDmaConfig != 0) || (pHandle->id == SYS_ETB_ID))
    {
        ret = flush(pHandle, eETB_STOP_FORMATTER);
    }
    else // Only Flush to the ETB, if EDMA mode is disabled
#endif
    {
        ret = flush(pHandle, eETB_OPT_NONE);
    }

    return ret;
}

/**
* flush - internal function used to flush the ETB from both the ETB_Disable API
*         and the ETB_flush API.
*/
static eETB_Error flush(ETBHandle *pHandle, eETB_OPTIONS options)
{
    eETB_Error  ret = eETB_Success;
    uint32_t etbControl;
    uint32_t status;
    uint32_t retry;
#if defined(C6670) || defined(C6678)

#ifdef DMA_SUPPORT
 
    uint32_t rrp;
    uint32_t cntr = 0;
   
    if(pHandle->pDmaConfig != 0)
    {
        // Check for both Stop On Full mode is enabled and ETBLib buffer is wrapped
        if((etbLib_bufferWrapped[pHandle->id][0] == 1) && (pHandle->pDmaConfig->mode == eDMA_Stop_Buffer))
        {
            // Re-enable the ETB trace as the ETB trace was disabled by EDMA on buffer full
            /* Enable ETB data capture by writing ETB Control Register. */
            *((volatile uint32_t*)ETB_CTL(pHandle->id)) |= ETB_ENABLE; /* TraceCaptEn =1 */
        }
    }

    /* If using DMA to drain the ETB, disable the event coming from the ETB
     *  interrupts in case the flush causes the ETB to cross the half-full or
     *  full threshold. If this were to occur and start a DMA transaction the
     *  data or PaRAM could get corrupted when the ETB_flush_dma function is
     *  called. After the flush occurs the read from the Event Register will
     *  identify if an interrupt occurred from the ETB.
     *
     * This code will only get executed on the 1st call, it is expected this
     *  function is called directly before calling the DMA flush function.
     *  Afterwards the ETB read pointer will not be on an even half or full
     *  boundary and the while loop below would timeout. The configuration
     *  function will need to get called before another capture can occur.
     */
    if((pHandle->pDmaConfig != 0) && (pHandle->dmaStatus.flushRequired))
    {
        uint32_t etbHalfSize;

        /* By this point the ETB read register should be stable and at 0 or half
         *  the ETB size, if not, then wait for the previous DMA to complete.
         */
        rrp = *((volatile uint32_t*)ETB_RRP(pHandle->id));
        etbHalfSize = *((volatile uint32_t*)ETB_RDP(pHandle->id));
        etbHalfSize /= 2;
        while((rrp != 0) && (rrp != etbHalfSize))
        {
            rrp = *((volatile uint32_t*)ETB_RRP(pHandle->id));
            if(cntr++ > ETB_DMA_TIMEOUT)
            	RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
        }

        /* Disable Event Register */
        if(pHandle->pDmaConfig->clrChannel > 31)
        {
            EDMA3_EECRH_REG(pHandle->pDmaConfig->cc) =
                                    (1 << (pHandle->pDmaConfig->clrChannel-32));
        }
        else
        {
            EDMA3_EECR_REG(pHandle->pDmaConfig->cc) =
                                    (1 << pHandle->pDmaConfig->clrChannel);
        }

        /* Clear Interrupt Pending register for ETB channel */
        if(pHandle->pDmaConfig->etbChannel > 31)
        {
            EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                (1 << (pHandle->pDmaConfig->etbChannel - 32));
        }
        else
        {
            EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                (1 << pHandle->pDmaConfig->etbChannel);
        }
    }

#endif //#ifdef DMA_SUPPORT
    
#elif defined(C6657) || defined(C66AK2Hxx)

#ifdef DMA_SUPPORT

    uint32_t rrp;
    uint32_t cntr = 0;
   
    if(pHandle->pDmaConfig != 0)
    {
        // Check for both Stop On Full mode is enabled and ETBLib buffer is wrapped
        if((etbLib_bufferWrapped[pHandle->id][0] == 1) && (pHandle->pDmaConfig->mode == eDMA_Stop_Buffer))
        {
            // Re-enable the ETB trace as the ETB trace was disabled by EDMA on buffer full
            /* Enable ETB data capture by writing ETB Control Register. */
            *((volatile uint32_t*)ETB_CTL(pHandle->id)) |= ETB_ENABLE; /* TraceCaptEn =1 */
        }
    }

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {

        /* If using DMA to drain the ETB, disable the event coming from the ETB
         *  interrupts in case the flush causes the ETB to cross the half-full or
         *  full threshold. If this were to occur and start a DMA transaction the
         *  data or PaRAM could get corrupted when the ETB_flush_dma function is
         *  called. After the flush occurs the read from the Event Register will
         *  identify if an interrupt occurred from the ETB.
         *
         * This code will only get executed on the 1st call, it is expected this
         *  function is called directly before calling the DMA flush function.
         *  Afterwards the ETB read pointer will not be on an even half or full
         *  boundary and the while loop below would timeout. The configuration
         *  function will need to get called before another capture can occur.
         */
        if((pHandle->pDmaConfig != 0) && (pHandle->dmaStatus.flushRequired))
        {
            uint32_t etbHalfSize;

            /* By this point the ETB read register should be stable and at 0 or half
             *  the ETB size, if not, then wait for the previous DMA to complete.
             */
            rrp = *((volatile uint32_t*)ETB_RRP(pHandle->id));
            etbHalfSize = *((volatile uint32_t*)ETB_RDP(pHandle->id));
            etbHalfSize /= 2;
            while((rrp != 0) && (rrp != etbHalfSize))
            {
                rrp = *((volatile uint32_t*)ETB_RRP(pHandle->id));
                if(cntr++ > ETB_DMA_TIMEOUT)
                	RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
            }

            /* Disable etbhalf Event Register */
            if(pHandle->pDmaConfig->etbhalfChannel > 31)
            {
                EDMA3_EECRH_REG(pHandle->pDmaConfig->cc) =
                                        (1 << (pHandle->pDmaConfig->etbhalfChannel-32));
            }
            else
            {
                EDMA3_EECR_REG(pHandle->pDmaConfig->cc) =
                                        (1 << pHandle->pDmaConfig->etbhalfChannel);
            }

            /* Disable etbfull Event Register */
            if(pHandle->pDmaConfig->etbfullChannel > 31)
            {
                EDMA3_EECRH_REG(pHandle->pDmaConfig->cc) =
                                        (1 << (pHandle->pDmaConfig->etbfullChannel-32));
            }
            else
            {
                EDMA3_EECR_REG(pHandle->pDmaConfig->cc) =
                                        (1 << pHandle->pDmaConfig->etbfullChannel);
            }

            /* Clear Interrupt Pending register for ETB half channel */
            if(pHandle->pDmaConfig->etbhalfChannel > 31)
            {
                EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                    (1 << (pHandle->pDmaConfig->etbhalfChannel - 32));
            }
            else
            {
                EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                    (1 << pHandle->pDmaConfig->etbhalfChannel);
            }
        }
    }
    
#endif //#ifdef DMA_SUPPORT
#endif //#elif defined(C6657) || defined(C66AK2Hxx)

    /* Flush ETB and DTF*/
    etbControl = *((volatile uint32_t*)ETB_FFCR(pHandle->id));

    if(options == eETB_STOP_FORMATTER)
    {
        etbControl |= (1 << 12);  /* Stop Formatter bit */
    }
    *((volatile uint32_t*)ETB_FFCR(pHandle->id)) = etbControl;

    etbControl |= (1<<6); /* Manual flush */
    *((volatile uint32_t*)ETB_FFCR(pHandle->id)) = etbControl;

    // Wait for the flush to complete
    retry = 1000;
    do
    {
       status = *(volatile uint32_t*)ETB_FFSR(pHandle->id);
       retry--;
    }  while( ( ( status & ETB_FLUSH_INPROGRESS ) != 0 ) && ( retry != 0 ) );

#if defined(C6670) || defined(C6678)

#ifdef DMA_SUPPORT

    /* If using the DMA to drain the ETB, will need to check if the flush caused
     *  a threshold interrupt from the ETB.
     */
    if((pHandle->pDmaConfig != 0) && (pHandle->dmaStatus.flushRequired))
    {
        uint32_t regValue;

        /* Read the event register, check for an event, then start and wait for
         *  the DMA to complete before continuing.
         */
        if(pHandle->pDmaConfig->clrChannel > 31)
        {
            regValue = EDMA3_ERH_REG(pHandle->pDmaConfig->cc) &
                        (1 << (pHandle->pDmaConfig->clrChannel-32));
            EDMA3_EESRH_REG(pHandle->pDmaConfig->cc) =
                                    (1 << (pHandle->pDmaConfig->clrChannel-32));
        }
        else
        {
            regValue = EDMA3_ER_REG(pHandle->pDmaConfig->cc) &
                            (1 << pHandle->pDmaConfig->clrChannel);
            EDMA3_EESR_REG(pHandle->pDmaConfig->cc) =
                                        (1 << pHandle->pDmaConfig->clrChannel);
        }

        if(regValue)
        {
            cntr = 0;
            if(pHandle->pDmaConfig->etbChannel > 31)
            {
                while((EDMA3_IPRH_REG(pHandle->pDmaConfig->cc) &
                        (1 << (pHandle->pDmaConfig->etbChannel - 32))) == 0)
                {
                    if(cntr++ > ETB_DMA_TIMEOUT)
                    	RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
                }
                EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                    (1 << (pHandle->pDmaConfig->etbChannel - 32));
            }
            else
            {
                while((EDMA3_IPR_REG(pHandle->pDmaConfig->cc) &
                        (1 << pHandle->pDmaConfig->etbChannel)) == 0)
                {
                    if(cntr++ > ETB_DMA_TIMEOUT)
                    	RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
                }
                EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                    (1 << pHandle->pDmaConfig->etbChannel);
            }
        }
    
        /* Clear flag indicating not to excercise DMA code for any consecutive calls
           to this function.
        */
        pHandle->dmaStatus.flushRequired = 0;
    }

#endif //#ifdef DMA_SUPPORT
    
#elif defined(C6657) || defined(C66AK2Hxx)

#ifdef DMA_SUPPORT

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        /* If using the DMA to drain the ETB, will need to check if the flush caused
         *  a threshold interrupt from the ETB.
         */
        if((pHandle->pDmaConfig != 0) && (pHandle->dmaStatus.flushRequired))
        {
            uint32_t regValue1, regValue2;

            /* Read the event register, check for a etbhalf or etbfull event, then start and wait for
             *  the DMA to complete before continuing.
             */
            if(pHandle->pDmaConfig->etbhalfChannel > 31)
            {
                regValue1 = EDMA3_ERH_REG(pHandle->pDmaConfig->cc) &
                            (1 << (pHandle->pDmaConfig->etbhalfChannel-32));
                EDMA3_EESRH_REG(pHandle->pDmaConfig->cc) =
                                        (1 << (pHandle->pDmaConfig->etbhalfChannel-32));
            }
            else
            {
                regValue1 = EDMA3_ER_REG(pHandle->pDmaConfig->cc) &
                                (1 << pHandle->pDmaConfig->etbhalfChannel);
                EDMA3_EESR_REG(pHandle->pDmaConfig->cc) =
                                            (1 << pHandle->pDmaConfig->etbhalfChannel);
            }

            if(pHandle->pDmaConfig->etbfullChannel > 31)
            {
                regValue2 = EDMA3_ERH_REG(pHandle->pDmaConfig->cc) &
                            (1 << (pHandle->pDmaConfig->etbfullChannel-32));
                EDMA3_EESRH_REG(pHandle->pDmaConfig->cc) =
                                        (1 << (pHandle->pDmaConfig->etbfullChannel-32));
            }
            else
            {
                regValue2 = EDMA3_ER_REG(pHandle->pDmaConfig->cc) &
                                (1 << pHandle->pDmaConfig->etbfullChannel);
                EDMA3_EESR_REG(pHandle->pDmaConfig->cc) =
                                            (1 << pHandle->pDmaConfig->etbfullChannel);
            }

            if((regValue1 != 0) || (regValue2 != 0))
            {
                cntr = 0;
                if(pHandle->pDmaConfig->etbhalfChannel > 31)
                {
                    while((EDMA3_IPRH_REG(pHandle->pDmaConfig->cc) &
                            (1 << (pHandle->pDmaConfig->etbhalfChannel - 32))) == 0)
                    {
                        if(cntr++ > ETB_DMA_TIMEOUT)
                        	RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
                    }
                    EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                        (1 << (pHandle->pDmaConfig->etbhalfChannel - 32));
                }
                else
                {
                    while((EDMA3_IPR_REG(pHandle->pDmaConfig->cc) &
                            (1 << pHandle->pDmaConfig->etbhalfChannel)) == 0)
                    {
                        if(cntr++ > ETB_DMA_TIMEOUT)
                        	RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
                    }
                    EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                        (1 << pHandle->pDmaConfig->etbhalfChannel);
                }
            }
        
            /* Clear flag indicating not to excercise DMA code for any consecutive calls
               to this function.
            */
            pHandle->dmaStatus.flushRequired = 0;
        }

        *((volatile uint32_t*)ETB_FFCR(pHandle->id)) = 0;
    }
    
#endif //#ifdef DMA_SUPPORT
#endif //#elif defined(C6657) || defined(C66AK2Hxx)

    if ( retry == 0 )
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    return ret;
}

#ifdef DMA_SUPPORT
/******************************************************************************/
/*! \copydoc ETB_config_dma
 */
eETB_Error  ETB_config_dma(ETBHandle* pHandle, const DMAConfig *pConfig)
{
#if defined(C6670) || defined(C6678)
    
    uint32_t paramBase;
    uint16_t paramIdx;
    uint32_t etbSize;
    uint32_t etbHalfSize;
    uint32_t clr1param;
    uint32_t clr2param;
    uint32_t etb1param;
    uint32_t etb2param;
    uint32_t etb3param;
    struct edma3_param param;
    /* Fixed location to store DMA configuration parameters */
    static DMAConfig dmaConfig[NUM_ETB_INSTANCES];

    // For C6678, Index = 0 (CIC2), Index = 1 (CIC3)
    // For C6670, Index = 0 (CIC1), Index = 1 (CIC2)
    uint8_t cic_index = 0;

    if(!pHandle || (pHandle->ulContext != ETB_UNLOCK_VAL) || !pConfig)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    if ((TI_ETB_TI_MODE & *((volatile uint32_t*)ETB_TI_CTL(pHandle->id))) != TI_ETB_TI_MODE )
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    #if defined(C6670)
    if(!(pConfig->cic == eCIC_1 || pConfig->cic == eCIC_2))
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    #elif defined(C6678)
    if(!(pConfig->cic == eCIC_2 || pConfig->cic == eCIC_3))
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
    
    #endif

    /* Validate the Channel Controller value and set the Parameter RAM base
     *  address value
     */
    switch(pConfig->cc)
    {
        case 0:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(0);
            break;
        case 1:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(1);
            break;
        case 2:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(2);
            break;
        default:
            RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    /* Calculate the address values for the specific PaRAM's. The PaRAM numbers
     *  for the DMA channels provided are determined by reading the DMA to
     *  PaRAM mapping registers.
     */
    paramIdx  = (EDMA3_DCHMAP_REG(pConfig->cc,pConfig->clrChannel) >> 5);
    clr1param = paramBase + (0x20 * paramIdx);
    clr2param = paramBase + (0x20 * pConfig->linkparam[0]);
    paramIdx  = (EDMA3_DCHMAP_REG(pConfig->cc,pConfig->etbChannel) >> 5);
    etb1param = paramBase + (0x20 * paramIdx);
    etb2param = paramBase + (0x20 * pConfig->linkparam[1]);
    etb3param = paramBase + (0x20 * pConfig->linkparam[2]);

    etbLib_bufferWrapped[pHandle->id][0] = 0;
    etbLib_bufferWrapped[pHandle->id][1] = 1;
    
    /* Store configuration information */
    dmaConfig[Handle_index(pHandle->id)].cc           = pConfig->cc;
    dmaConfig[Handle_index(pHandle->id)].clrChannel   = pConfig->clrChannel;
    dmaConfig[Handle_index(pHandle->id)].etbChannel   = pConfig->etbChannel;
    dmaConfig[Handle_index(pHandle->id)].linkparam[0] = pConfig->linkparam[0];
    dmaConfig[Handle_index(pHandle->id)].linkparam[1] = pConfig->linkparam[1];
    dmaConfig[Handle_index(pHandle->id)].linkparam[2] = pConfig->linkparam[2];
    dmaConfig[Handle_index(pHandle->id)].dbufAddress  = pConfig->dbufAddress;
    dmaConfig[Handle_index(pHandle->id)].dbufWords    = pConfig->dbufWords;
    dmaConfig[Handle_index(pHandle->id)].mode         = pConfig->mode;
    dmaConfig[Handle_index(pHandle->id)].cic          = pConfig->cic;
    pHandle->pDmaConfig = &dmaConfig[Handle_index(pHandle->id)];

    /* Get the size of the ETB for this instance to determine the count values
     *  required for the EDMA parameter ram (PaRAM) configuration. The size if
     *  provided in words, convert to bytes for calculations. The count values
     *  for the EDMA are in bytes.
     */
    etbSize = *((volatile uint32_t*)ETB_RDP(pHandle->id));
    etbSize *= 4;
    etbHalfSize = etbSize / 2;

    #if defined(C6670)

    cic_index = pHandle->pDmaConfig->cic - 1;

    #elif defined(C6678)

    cic_index = pHandle->pDmaConfig->cic - 2;

    #endif

    /* The following 2 PaRAM's are used to clear the INTC1 event register. When
     *  the bit for the specific event in the register is set, no other incoming
     *  events will get forwarded to its corresponding output until the event
     *  has been cleared.
     * The early completion is used to chain to the start of the ETB transfer
     *  PaRAM.
     */
    param.options   = (PARAM_OPT_TCCHEN |
                       PARAM_OPT_TCC(pConfig->etbChannel) |
                       PARAM_OPT_TCC_EARLY|
                       PARAM_OPT_AB_SYNC);
    param.src_addr  = GET_GLOBAL_ADDR(&etbLib_cpCicEventClearValue[cic_index][pHandle->id][0]);
    param.ab_cnt    = (PARAM_BCNT(2) | PARAM_ACNT(4));
    param.dst_addr  = etbLib_cpCicEventClearIndexReg[cic_index];
    param.srcdst_bidx = 4;
    param.link_bcnt = PARAM_LINK(clr2param);
    param.srcdst_cidx = 0;
    param.ccnt        = 1;

    /* Copy local structure to actual PaRAM memory locations */
    *(struct edma3_param *)clr1param = param;
    *(struct edma3_param *)clr2param = param;

    /* Create the PaRAM entries that will be used to transfer data from ETB RAM
     *  Burst Data Read Register locations to a drain buffer in memory. The
     *  second PaRAM is used once to transfer a buffer wrapped flag that is used
     *  to indicate the drain buffer has wrapped. It is linked to the 3rd PaRAM
     *  that is used to reload with initial values to tranfer data from ETB.
     */
    param.options   = (PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN |
                       PARAM_OPT_TCCHEN   |
                       PARAM_OPT_TCC(pConfig->etbChannel) |
                       PARAM_OPT_AB_SYNC);
    param.src_addr  = ETB_RBD(pHandle->id);
    param.ab_cnt    = PARAM_BCNT(etbHalfSize / ETB_BURST_SIZE) | PARAM_ACNT(ETB_BURST_SIZE);
    param.dst_addr  = pConfig->dbufAddress;
    param.srcdst_bidx = PARAM_DST_BIDX(ETB_BURST_SIZE);
    param.link_bcnt   = PARAM_LINK(etb2param);
    param.srcdst_cidx = PARAM_DST_CIDX(etbHalfSize);
    param.ccnt        = (pConfig->dbufWords*4) / etbHalfSize;

    /* Set the DMA status to the actual number of words used in the provided
     *  buffer, also reset any previous status settings.
     */
    pHandle->dmaStatus.startAddr      = 0;
    pHandle->dmaStatus.availableWords = 0;
    pHandle->dmaStatus.isWrapped      = 0;
    pHandle->dmaStatus.dbufAddress    = 0;
    pHandle->dmaStatus.flushRequired  = 1;
    pHandle->dmaStatus.dbufWords = (param.ccnt * etbHalfSize) / 4;

    /* Copy local structure to actual PaRAM memory locations */
    *(struct edma3_param *)etb1param = param;

    if(pConfig->mode != eDMA_Stop_Buffer)
    {

        /* Change the 3rd PaRAM to not link to 2nd for wrap processing, just
         *  link to self for reloading purposes, Transfer chaining completion is
         *  also not needed except for the 1st occurance.
         */
        param.options   = PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN | PARAM_OPT_AB_SYNC;
        param.link_bcnt = PARAM_LINK(etb3param);
    }
    else
    {
        /* If the DMA mode has been configured as non-circular, configure etbparam3 to disable ETB trace
         *  set the link value to a NULL after disabling ETB trace.
         */
         
        param.options   = PARAM_OPT_TCC_EARLY;
        param.src_addr  = GET_GLOBAL_ADDR(&etb_disable);
        param.ab_cnt    = (PARAM_BCNT(1) | PARAM_ACNT(4));
        param.dst_addr  = ETB_CTL(pHandle->id);
        param.srcdst_bidx = 0;
        param.link_bcnt = PARAM_LINK(0xffff);
        param.srcdst_cidx = 0;
        param.ccnt        = 1;
    }
    
    *(struct edma3_param *)etb3param = param;
    
    if(pConfig->mode != eDMA_Stop_Buffer)
    { 
        /* 2nd PaRAM configuration, no transfer chaining completion, but an early
         *  link completion.
         */
        param.options   = PARAM_OPT_TCC_EARLY;
    }
    else
    {
        /* For Stop on full buffer mode, link to etb3param which disables ETB trace capture */
        param.options   = (PARAM_OPT_TCC_EARLY | PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN |
                           PARAM_OPT_TCCHEN   |
                           PARAM_OPT_TCC(pConfig->etbChannel));
    } 
                   
    param.src_addr  = GET_GLOBAL_ADDR(&etbLib_bufferWrapped[pHandle->id][1]);
    param.ab_cnt    = PARAM_BCNT(1) | PARAM_ACNT(4);
    param.dst_addr  = GET_GLOBAL_ADDR(&etbLib_bufferWrapped[pHandle->id][0]);
    param.srcdst_bidx = 0;
    param.link_bcnt   = PARAM_LINK(etb3param);
    param.srcdst_cidx = 0;
    param.ccnt        = 1;

    /* Copy local structure to actual PaRAM memory locations */
    *(struct edma3_param *)etb2param = param;

    /* Enable event for specific channel that is used to clear the INTCx
     *  interrupt status. This is the EDMA event that is used to start the DMA
     *  transactions.
     */
    if(pConfig->clrChannel > 31)
    {
        EDMA3_EESRH_REG(pConfig->cc) = (1 << (pConfig->clrChannel-32));
    }
    else
    {
        EDMA3_EESR_REG(pConfig->cc) = (1 << pConfig->clrChannel);
    }
    
#elif defined(C6657) || defined(C66AK2Hxx)
    
    uint32_t paramBase;
    uint32_t etbSize;
    uint32_t etbHalfSize;
    uint32_t etb1param;
    uint32_t etb2param;
    uint32_t etb3param;
    struct edma3_param param;
#ifdef C66AK2Hxx
    int dev_index;
    uint32_t jtag_id;
#endif
    
    /* Fixed location to store DMA configuration parameters */
    static DMAConfigInt dmaConfig[NUM_ETB_INSTANCES];

    if(!pHandle || (pHandle->ulContext != ETB_UNLOCK_VAL) || !pConfig)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        if ((TI_ETB_TI_MODE & *((volatile uint32_t*)ETB_TI_CTL(pHandle->id))) != TI_ETB_TI_MODE )
        {
            RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
        }
    }

    /* Store configuration information */
    
#if defined(C6657)
    
    dmaConfig[Handle_index(pHandle->id)].cc           = 2; //The ETB half-full and full events are directly connected to EDMA CC in C6657
    if(pHandle->id == SYS_ETB_ID)
    {
        dmaConfig[Handle_index(pHandle->id)].etbhalfChannel   = EDMACC_TETBHFULLINT; //ETB half-full event
        dmaConfig[Handle_index(pHandle->id)].etbfullChannel   = EDMACC_TETBFULLINT; //ETB full event
    }
    else
    {
        dmaConfig[Handle_index(pHandle->id)].etbhalfChannel   = EDMACC_TETBHFULLINT0 + pHandle->id; //ETB half-full event
        dmaConfig[Handle_index(pHandle->id)].etbfullChannel   = EDMACC_TETBFULLINT0 + pHandle->id; //ETB full event
    }
    
#endif

#if defined(C66AK2Hxx)

    /* Get the JTAG ID and get the device id */
    jtag_id = *(volatile uint32_t *)JTAG_ID_REG & JTAG_ID_MASK;

    for (dev_index = 0; dev_index <  keystone2_jtagId_table_elements; dev_index++ )
    {
    	if (jtag_id == keystone2_jtagId_table[dev_index])
    	{
    		break;
    	}
    }
    if (dev_index == keystone2_jtagId_table_elements)
    {
    	return eETB_Error_Program;
    }

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        dmaConfig[Handle_index(pHandle->id)].cc =  keystone2_edma_config_table[dev_index][pHandle->id].cc_id;
        dmaConfig[Handle_index(pHandle->id)].etbhalfChannel   =  keystone2_edma_config_table[dev_index][pHandle->id].etbhalfChannel; //ETB half-full event
        dmaConfig[Handle_index(pHandle->id)].etbfullChannel   =  keystone2_edma_config_table[dev_index][pHandle->id].etbfullChannel; //ETB full event

    } else {
        /* Must be a TBR */

        //Setup SYS ETB or CSSTM ETB for DMA drain mode
        //TBR system bridge operation

        // Setup TBR in system bridge (DMA) mode
        *((volatile uint32_t*)ETB_CTL(pHandle->id)) |= TBR_BRIDGE_MODE;

        dmaConfig[Handle_index(pHandle->id)].cc = keystone2_edma_config_table[dev_index][pHandle->id].cc_id;
        dmaConfig[Handle_index(pHandle->id)].etbhalfChannel = keystone2_edma_config_table[dev_index][pHandle->id].etbhalfChannel;

        //Setup DMA trigger thresholds
        if(Handle_index(pHandle->id) == SYS_ETB_ID)
        {
            *((volatile uint32_t*)TBR_OUTLVL(pHandle->id)) = ((SYS_TBR_NUMBLOCK << 8) | SYS_TBR_BLOCKSZ);
        }
        else if(Handle_index(pHandle->id) == ARM_ETB_ID)

        {
            *((volatile uint32_t*)TBR_OUTLVL(pHandle->id)) = ((ARM_TBR_NUMBLOCK << 8) | ARM_TBR_BLOCKSZ);
        }


        //Enable DMA trigger
        *((volatile uint32_t*)TBR_IRQENABLE_SET(pHandle->id)) = TBR_IRST_DAV;


    }
#endif //#if defined(C66AK2Hxx)
    
    dmaConfig[Handle_index(pHandle->id)].linkparam[0] = pConfig->linkparam[0];
    dmaConfig[Handle_index(pHandle->id)].linkparam[1] = pConfig->linkparam[1];
    dmaConfig[Handle_index(pHandle->id)].linkparam[2] = pConfig->linkparam[2];
    dmaConfig[Handle_index(pHandle->id)].dbufAddress  = pConfig->dbufAddress;
    dmaConfig[Handle_index(pHandle->id)].dbufWords    = pConfig->dbufWords;
    dmaConfig[Handle_index(pHandle->id)].mode         = pConfig->mode;
    pHandle->pDmaConfig = &dmaConfig[Handle_index(pHandle->id)];
    
    /* Validate the Channel Controller value and set the Parameter RAM base
     *  address value
     */
    switch(pHandle->pDmaConfig->cc)
    {
        case 0:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(0);
            break;
        case 1:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(1);
            break;
        case 2:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(2);
            break;
        case 3:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(3);
            break;
        case 4:
            paramBase = EDMA_TPCC_PARAM_BASE_ADDR(4);
            break;
        default:
            RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
    
    // Mapping linkparam[0] to the ETB half channel
    EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,pHandle->pDmaConfig->etbhalfChannel) = (pHandle->pDmaConfig->linkparam[0] << 5);

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        // Mapping ETB full channel is only required for TI-ETB
        // Mapping linkparam[0] to the ETB full channel
        EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,pHandle->pDmaConfig->etbfullChannel) = (pHandle->pDmaConfig->linkparam[0] << 5);
    }

    /* Calculate the param address for etb1param, etb2param and etb3param
     */

    etb1param = paramBase + (0x20 * pConfig->linkparam[0]);
    etb2param = paramBase + (0x20 * pConfig->linkparam[1]);
    etb3param = paramBase + (0x20 * pConfig->linkparam[2]);

    etbLib_bufferWrapped[pHandle->id][0] = 0;
    etbLib_bufferWrapped[pHandle->id][1] = 1;

    /* Create the PaRAM entries that will be used to transfer data from ETB RAM
     *  Burst Data Read Register locations to a drain buffer in memory. The
     *  second PaRAM is used once to transfer a buffer wrapped flag that is used
     *  to indicate the drain buffer has wrapped. It is linked to the 3rd PaRAM
     *  that is used to reload with initial values to transfer data from ETB.
     */
    param.options   = (PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN |
                       PARAM_OPT_TCCHEN   |
                       PARAM_OPT_TCC(pHandle->pDmaConfig->etbhalfChannel) |
                       PARAM_OPT_AB_SYNC);

    /* Get the size of the ETB for this instance to determine the count values
     *  required for the EDMA parameter ram (PaRAM) configuration. The size if
     *  provided in words, convert to bytes for calculations. The count values
     *  for the EDMA are in bytes.
     */

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        // get ETB buffer size
        etbSize = *((volatile uint32_t*)ETB_RDP(pHandle->id));

        // get starting address of the ETB buffer
        param.src_addr  = ETB_RBD(pHandle->id);
    }
    else
    {

#if defined(C66AK2Hxx)
        // get TBR buffer size
        etbSize = (*((volatile uint32_t*)ETB_RDP(pHandle->id)));
        etbSize = (1 << (etbSize-1)) << 10;

        // get starting address of the TBR buffer
        if(Handle_index(pHandle->id) == ARM_ETB_ID)
        {
            param.src_addr  = ARM_TBR_RBD;
        }
        else if(Handle_index(pHandle->id) == SYS_ETB_ID)
        {
            param.src_addr  = SYS_TBR_RBD;
        }
#endif
    }

    etbSize *= 4;
    etbHalfSize = etbSize / 2;

    param.ab_cnt    = PARAM_BCNT(etbHalfSize / ETB_BURST_SIZE) | PARAM_ACNT(ETB_BURST_SIZE);
    param.dst_addr  = pConfig->dbufAddress;
    param.srcdst_bidx = PARAM_DST_BIDX(ETB_BURST_SIZE);
    param.link_bcnt   = PARAM_LINK(etb2param);
    param.srcdst_cidx = PARAM_DST_CIDX(etbHalfSize);
    param.ccnt        = (pConfig->dbufWords*4) / etbHalfSize;

    /* Set the DMA status to the actual number of words used in the provided
     *  buffer, also reset any previous status settings.
     */
    pHandle->dmaStatus.startAddr      = 0;
    pHandle->dmaStatus.availableWords = 0;
    pHandle->dmaStatus.isWrapped      = 0;
    pHandle->dmaStatus.dbufAddress    = 0;
    pHandle->dmaStatus.flushRequired  = 1;
    pHandle->dmaStatus.dbufWords = (param.ccnt * etbHalfSize) / 4;

    /* Copy local structure to actual PaRAM memory locations */
    *(struct edma3_param *)etb1param = param;

    if(pConfig->mode != eDMA_Stop_Buffer)
    {

        /* Change the 3rd PaRAM to not link to 2nd for wrap processing, just
         *  link to self for reloading purposes, Transfer chaining completion is
         *  also not needed except for the 1st occurance.
         */
        param.options   = PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN | PARAM_OPT_AB_SYNC;
        param.link_bcnt = PARAM_LINK(etb3param);
    }
    else
    {
        if(check_etb_type(pHandle->id) == ETB_TYPE)
		{
			/* If the DMA mode has been configured as non-circular, configure etbparam3 to disable ETB trace
			 *  set the link value to a NULL after disabling ETB trace.
			 */
			 
			param.options   = PARAM_OPT_TCC_EARLY;
			param.src_addr  = GET_GLOBAL_ADDR(&etb_disable);
			param.ab_cnt    = (PARAM_BCNT(1) | PARAM_ACNT(4));
			param.dst_addr  = ETB_CTL(pHandle->id);
			param.srcdst_bidx = 0;
			param.link_bcnt = PARAM_LINK(0xffff);
			param.srcdst_cidx = 0;
			param.ccnt        = 1;
		}
		else
		{

#if defined(C66AK2Hxx)
			//For TBR implementations
			//Move trace data to a dummy buffer whenever stop on full condition is met
			//This is done to make sure the trace sources filling up the TBR are not stalled indefinitely
			param.options   = PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN | PARAM_OPT_AB_SYNC;
			param.dst_addr  = GET_GLOBAL_ADDR(&etb_stop_on_full_tempBuffer);
			param.srcdst_bidx = 0;
			param.link_bcnt = PARAM_LINK(etb3param);
			param.srcdst_cidx = 0;
			param.ccnt        = 1;
#endif
		}
    }
    
    *(struct edma3_param *)etb3param = param;
    
    if(pConfig->mode != eDMA_Stop_Buffer)
    { 
        /* 2nd PaRAM configuration, no transfer chaining completion, but an early
         *  link completion.
         */
        param.options   = PARAM_OPT_TCC_EARLY;
    }
    else
    {
    	if(check_etb_type(pHandle->id) == ETB_TYPE)
    	{
        	/* For Stop on full buffer mode, link to etb3param which disables ETB trace capture */
            param.options   = (PARAM_OPT_TCC_EARLY | PARAM_OPT_ITCINTEN | PARAM_OPT_TCINTEN |
                               PARAM_OPT_TCCHEN   |
                               PARAM_OPT_TCC(pHandle->pDmaConfig->etbhalfChannel));
    	}
    	else //TBR implementation
    	{
            /* 2nd PaRAM configuration, no transfer chaining completion, but an early
             *  link completion.
             */
            param.options   = PARAM_OPT_TCC_EARLY;
    	}
    } 
                   
    param.src_addr  = GET_GLOBAL_ADDR(&etbLib_bufferWrapped[pHandle->id][1]);
    param.ab_cnt    = PARAM_BCNT(1) | PARAM_ACNT(4);
    param.dst_addr  = GET_GLOBAL_ADDR(&etbLib_bufferWrapped[pHandle->id][0]);
    param.srcdst_bidx = 0;
    param.link_bcnt   = PARAM_LINK(etb3param);
    param.srcdst_cidx = 0;
    param.ccnt        = 1;

    /* Copy local structure to actual PaRAM memory locations */
    *(struct edma3_param *)etb2param = param;

    /* Enable event for etb half and etb full channel that. This is the EDMA event that is used to start the DMA
     *  transactions.
     */
    if(pHandle->pDmaConfig->etbhalfChannel > 31)
    {
        EDMA3_EESRH_REG(pHandle->pDmaConfig->cc) = (1 << (pHandle->pDmaConfig->etbhalfChannel-32));
    }
    else
    {
        EDMA3_EESR_REG(pHandle->pDmaConfig->cc) = (1 << pHandle->pDmaConfig->etbhalfChannel);
    }
    
    if(pHandle->pDmaConfig->etbfullChannel > 31)
    {
        EDMA3_EESRH_REG(pHandle->pDmaConfig->cc) = (1 << (pHandle->pDmaConfig->etbfullChannel-32));
    }
    else
    {
        EDMA3_EESR_REG(pHandle->pDmaConfig->cc) = (1 << pHandle->pDmaConfig->etbfullChannel);
    }
    
#endif //#elif defined(C6657) || defined(C66AK2Hxx)

    return eETB_Success;
}

/******************************************************************************/
/*! \copydoc ETB_flush_dma
 */
eETB_Error  ETB_flush_dma(ETBHandle* pHandle, DMAStatus *pStatus)
{
#if defined(C6670) || defined(C6678)
    
    int32_t remWords;
    uint32_t rwp;
    uint32_t rrp;
    uint16_t paramIdx;
    uint32_t paramAddress;
    volatile uint32_t cntr = 0;

    // For C6678, Index = 0 (CIC2), Index = 1 (CIC3)
    // For C6670, Index = 0 (CIC1), Index = 1 (CIC2)
    uint8_t cic_index = 0;

    if(!pHandle || !pStatus)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
    if(!pHandle->pDmaConfig)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

#if defined(C6670)
    if(!(pHandle->pDmaConfig->cic == eCIC_1 || pHandle->pDmaConfig->cic == eCIC_2))
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
#elif defined(C6678)
    if(!(pHandle->pDmaConfig->cic == eCIC_2 || pHandle->pDmaConfig->cic == eCIC_3))
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
    
#endif

#if defined(C6670)

    cic_index = pHandle->pDmaConfig->cic - 1;

#elif defined(C6678)

    cic_index = pHandle->pDmaConfig->cic - 2;

#endif

    rwp = *((volatile uint32_t*)ETB_RWP(pHandle->id));
    rrp = *((volatile uint32_t*)ETB_RRP(pHandle->id));

    remWords = rwp - rrp;
//    printf(" rwp: %x, rrp: %x, diff: %d\n", rwp, rrp, remWords);

    if(remWords > 0)
    {
        int32_t remBytes;
        uint16_t clrParamIdx;

        remBytes = (remWords * 4);
        /* Change the PaRAM set value for the DMA channel that handles the ETB
         *  event to the drain PaRAM. This method is more efficient than copying
         *  the PaRAM contents.
         */
        paramIdx = (EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                                     pHandle->pDmaConfig->etbChannel) >> 5);
        clrParamIdx = (EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                                     pHandle->pDmaConfig->clrChannel) >> 5);
        EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                         pHandle->pDmaConfig->clrChannel) = (paramIdx<<5);
        paramAddress = EDMA_TPCC_PARAM_BASE_ADDR(pHandle->pDmaConfig->cc) +
                                                    (0x20 * paramIdx);

        /* Check for buffer wrap */
        if(etbLib_bufferWrapped[pHandle->id][0])
        {
            pStatus->availableWords    = pHandle->dmaStatus.dbufWords;
            pStatus->startAddr = PARAM_DST_REG(paramAddress) + remBytes;
            pStatus->isWrapped = 1;
        }
        else
        {
            pStatus->availableWords = (PARAM_DST_REG(paramAddress) -
                                pHandle->pDmaConfig->dbufAddress) / 4;
            pStatus->availableWords += remWords;
            pStatus->startAddr = pHandle->pDmaConfig->dbufAddress;
            pStatus->isWrapped = 0;
        }

        /* Copy the DMA status into the handle for later management. The status
         *  value for the number of words may be different than what was in the
         *  configuration structure. This value is set during configuration.
         */
        pStatus->dbufAddress = pHandle->pDmaConfig->dbufAddress;
        pStatus->dbufWords   = pHandle->dmaStatus.dbufWords;
        pStatus->flushRequired = pHandle->dmaStatus.flushRequired;
        pHandle->dmaStatus   = *pStatus;

        /* If the mode has been set to non-circular and the buffer wrapped flag
         *  is set, then the memory buffer if full, otherwise continue the
         *  configuration for the final DMA.
         */
        if((pHandle->pDmaConfig->mode == eDMA_Stop_Buffer) &&
            (etbLib_bufferWrapped[pHandle->id][0]))
        {
            /* If the buffer wrapped, the startAddr value is incorrect from
             *  above, set back to beginning of the buffer.
             */
            pStatus->startAddr = pHandle->pDmaConfig->dbufAddress;
            pHandle->dmaStatus.startAddr = pHandle->pDmaConfig->dbufAddress;
            return eETB_Success;
        }
        
        // Update the 3 symbols which are required for CCS ETB receiver
        etbLib_buffer_start_addr[pHandle->id] = pHandle->pDmaConfig->dbufAddress; //CCS ETB receiver will always get a linearized buffer for the non-EDMA ETB drain case
        etbLib_buffer_size[pHandle->id] = pStatus->availableWords * 4; //Number of bytes available 
        etbLib_buffer_data_start[pHandle->id] = pStatus->startAddr; //circular buffer wrap point

        if(remBytes > ETB_BURST_SIZE)
        {
            uint16_t transCnt;
            
            /* Split the transfer into burst transactions.
             *  Any remaining data beyond an even multiple of burst size bytes
             *  will get linked for a single remaining transaction.
             */
            transCnt = remBytes / ETB_BURST_SIZE;
            PARAM_AB_CNT_REG(paramAddress) &= ~PARAM_BCNT_MASK;
            PARAM_AB_CNT_REG(paramAddress) |= PARAM_BCNT(transCnt);
            PARAM_CCNT_REG(paramAddress) = 1;
            
            /* Adjust the remaining byte count */
            remBytes -= (transCnt * ETB_BURST_SIZE);
        }
        else
        {
            PARAM_AB_CNT_REG(paramAddress) = PARAM_BCNT(1) | PARAM_ACNT(remBytes);
            PARAM_CCNT_REG(paramAddress) = 1;
            remBytes = 0;
        }
        /* If there are remaining bytes to transfer, setup link param for final
         *  transaction.
         */
        if(remBytes > 0)
        {
            uint32_t etb3param;

            etb3param = EDMA_TPCC_PARAM_BASE_ADDR(pHandle->pDmaConfig->cc) +
                        (0x20 * pHandle->pDmaConfig->linkparam[2]);
            
            /* Further ETB PaRAM modifications */
            PARAM_OPT_REG(paramAddress) = (PARAM_OPT_TCCHEN |
                                PARAM_OPT_TCC(pHandle->pDmaConfig->clrChannel) |
                                PARAM_OPT_AB_SYNC);
            PARAM_LINK_REG(paramAddress) &= ~PARAM_LINK_MASK;
            PARAM_LINK_REG(paramAddress) |= PARAM_LINK(etb3param);
            
            /* ETB linked PaRAM modifications */
            PARAM_OPT_REG(etb3param) = (PARAM_OPT_TCINTEN |
                                PARAM_OPT_TCC(pHandle->pDmaConfig->clrChannel) |
                                PARAM_OPT_AB_SYNC);
            PARAM_AB_CNT_REG(etb3param) = PARAM_BCNT(1) | PARAM_ACNT(remBytes);
            PARAM_DST_REG(etb3param)  = PARAM_DST_REG(paramAddress) +
                                        (PARAM_BCNT_VALUE(paramAddress) * ETB_BURST_SIZE);
            PARAM_CCNT_REG(etb3param) = 1;
            PARAM_LINK_REG(etb3param) &= ~PARAM_LINK_MASK;
            PARAM_LINK_REG(etb3param) |= PARAM_LINK(0xffff);
        }
        else
        {
            PARAM_OPT_REG(paramAddress) = (PARAM_OPT_TCINTEN |
                                PARAM_OPT_TCC(pHandle->pDmaConfig->clrChannel) |
                                PARAM_OPT_AB_SYNC);
        }

        CIC_STATUS_CLR_INDEX_REG(pHandle->pDmaConfig->cic) = etbLib_cpCicEventClearValue[cic_index][pHandle->id][0];
        CIC_STATUS_CLR_INDEX_REG(pHandle->pDmaConfig->cic) = etbLib_cpCicEventClearValue[cic_index][pHandle->id][1];

        if(pHandle->pDmaConfig->clrChannel > 31)
        {
            EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                (1 << (pHandle->pDmaConfig->clrChannel - 32));
        }
        else
        {
            EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                (1 << pHandle->pDmaConfig->clrChannel);
        }

        /* Force an event manually from the ETB interrupt register */
        *((volatile uint32_t*)ETB_IRST(pHandle->id)) = 1;

        /* Poll the interrupt pending bit for transaction completion */
        cntr = 0;
        if(pHandle->pDmaConfig->clrChannel > 31)
        {
            while((EDMA3_IPRH_REG(pHandle->pDmaConfig->cc) &
                    (1 << (pHandle->pDmaConfig->clrChannel - 32))) == 0)
            {
                if(cntr++ > ETB_DMA_TIMEOUT)
                    break;
            }

            CIC_STATUS_CLR_INDEX_REG(pHandle->pDmaConfig->cic) = etbLib_cpCicEventClearValue[cic_index][pHandle->id][0];
            CIC_STATUS_CLR_INDEX_REG(pHandle->pDmaConfig->cic) = etbLib_cpCicEventClearValue[cic_index][pHandle->id][1];

            EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                (1 << (pHandle->pDmaConfig->clrChannel - 32));
        }
        else
        {
            while((EDMA3_IPR_REG(pHandle->pDmaConfig->cc) &
                    (1 << pHandle->pDmaConfig->clrChannel)) == 0)
            {
                if(cntr++ > ETB_DMA_TIMEOUT)
                    break;
            }

            CIC_STATUS_CLR_INDEX_REG(pHandle->pDmaConfig->cic) = etbLib_cpCicEventClearValue[cic_index][pHandle->id][0];
            CIC_STATUS_CLR_INDEX_REG(pHandle->pDmaConfig->cic) = etbLib_cpCicEventClearValue[cic_index][pHandle->id][1];

            EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                (1 << pHandle->pDmaConfig->clrChannel);
        }
        
        /* Restore DMA Channel map for Clear Channel PaRAM index */
        EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                         pHandle->pDmaConfig->clrChannel) = (clrParamIdx<<5);
        
 //       printf("Pending Cntr: %d\n", cntr);
    }
    else if(remWords < 0)
    {
//        printf(" *** remWords < 0, %d\n",remWords);
        /* Report error if < 0, otherwise nothing to read from ETB */
        return eETB_Overflow;

    }
    
#elif defined(C6657) || defined(C66AK2Hxx)

    int32_t remWords, remBytes;
    uint32_t rwp;
    uint32_t rrp;
    uint32_t paramAddress;
    uint16_t paramIdx;
    uint32_t retry, status;
    volatile uint32_t cntr = 0;

    if(!pHandle || !pStatus)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }
    if(!pHandle->pDmaConfig)
    {
        RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
    }

    /* Change the PaRAM set value for the DMA channel that handles the ETB
     *  event to the drain PaRAM. This method is more efficient than copying
     *  the PaRAM contents.
     */
    paramIdx = (EDMA3_DCHMAP_REG(pHandle->pDmaConfig->cc,
                                 pHandle->pDmaConfig->etbhalfChannel) >> 5);
    paramAddress = EDMA_TPCC_PARAM_BASE_ADDR(pHandle->pDmaConfig->cc) +
                                                (0x20 * paramIdx);

    if(check_etb_type(pHandle->id) == ETB_TYPE)
    {
        rwp = *((volatile uint32_t*)ETB_RWP(pHandle->id));
        rrp = *((volatile uint32_t*)ETB_RRP(pHandle->id));

        remWords = rwp - rrp;
    //    printf(" rwp: %x, rrp: %x, diff: %d\n", rwp, rrp, remWords);

        if(remWords > 0)
        {
            remBytes = (remWords * 4);

            //Get EDMA buffer information
            if(get_edma_buffer_info(pHandle, pStatus, paramAddress, remWords))
            {
                return eETB_Success;
            }

            if(remBytes > ETB_BURST_SIZE)
            {
                uint16_t transCnt;

                /* Split the transfer into burst transactions.
                 *  Any remaining data beyond an even multiple of burst size bytes
                 *  will get linked for a single remaining transaction.
                 */
                transCnt = remBytes / ETB_BURST_SIZE;
                PARAM_AB_CNT_REG(paramAddress) &= ~PARAM_BCNT_MASK;
                PARAM_AB_CNT_REG(paramAddress) |= PARAM_BCNT(transCnt);
                PARAM_CCNT_REG(paramAddress) = 1;

                /* Adjust the remaining byte count */
                remBytes -= (transCnt * ETB_BURST_SIZE);
            }
            else
            {
                PARAM_AB_CNT_REG(paramAddress) = PARAM_BCNT(1) | PARAM_ACNT(remBytes);
                PARAM_CCNT_REG(paramAddress) = 1;
                remBytes = 0;
            }
            /* If there are remaining bytes to transfer, setup link param for final
             *  transaction.
             */
            if(remBytes > 0)
            {
                uint32_t etb3param;

                etb3param = EDMA_TPCC_PARAM_BASE_ADDR(pHandle->pDmaConfig->cc) +
                            (0x20 * pHandle->pDmaConfig->linkparam[2]);

                /* Further ETB PaRAM modifications */
                PARAM_OPT_REG(paramAddress) = (PARAM_OPT_TCCHEN |
                                    PARAM_OPT_TCC(pHandle->pDmaConfig->etbhalfChannel) |
                                    PARAM_OPT_AB_SYNC);
                PARAM_LINK_REG(paramAddress) &= ~PARAM_LINK_MASK;
                PARAM_LINK_REG(paramAddress) |= PARAM_LINK(etb3param);

                /* ETB linked PaRAM modifications */
                PARAM_OPT_REG(etb3param) = (PARAM_OPT_TCINTEN |
                                    PARAM_OPT_TCC(pHandle->pDmaConfig->etbhalfChannel) |
                                    PARAM_OPT_AB_SYNC);
                PARAM_AB_CNT_REG(etb3param) = PARAM_BCNT(1) | PARAM_ACNT(remBytes);
                PARAM_DST_REG(etb3param)  = PARAM_DST_REG(paramAddress) +
                                            (PARAM_BCNT_VALUE(paramAddress) * ETB_BURST_SIZE);
                PARAM_CCNT_REG(etb3param) = 1;
                PARAM_LINK_REG(etb3param) &= ~PARAM_LINK_MASK;
                PARAM_LINK_REG(etb3param) |= PARAM_LINK(0xffff);
            }
            else
            {
                PARAM_OPT_REG(paramAddress) = (PARAM_OPT_TCINTEN |
                                    PARAM_OPT_TCC(pHandle->pDmaConfig->etbhalfChannel) |
                                    PARAM_OPT_AB_SYNC);
            }

            if(pHandle->pDmaConfig->etbhalfChannel > 31)
            {
                EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                    (1 << (pHandle->pDmaConfig->etbhalfChannel - 32));
            }
            else
            {
                EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                    (1 << pHandle->pDmaConfig->etbhalfChannel);
            }

            /* Force an event manually from the ETB interrupt register */
            *((volatile uint32_t*)ETB_IRST(pHandle->id)) = 1;

            /* Poll the interrupt pending bit for transaction completion */
            cntr = 0;
            if(pHandle->pDmaConfig->etbhalfChannel > 31)
            {
                while((EDMA3_IPRH_REG(pHandle->pDmaConfig->cc) &
                        (1 << (pHandle->pDmaConfig->etbhalfChannel - 32))) == 0)
                {
                    if(cntr++ > ETB_DMA_TIMEOUT)
                        break;
                }

                EDMA3_ICRH_REG(pHandle->pDmaConfig->cc) =
                                    (1 << (pHandle->pDmaConfig->etbhalfChannel - 32));
            }
            else
            {
                while((EDMA3_IPR_REG(pHandle->pDmaConfig->cc) &
                        (1 << pHandle->pDmaConfig->etbhalfChannel)) == 0)
                {
                    if(cntr++ > ETB_DMA_TIMEOUT)
                        break;
                }

                EDMA3_ICR_REG(pHandle->pDmaConfig->cc) =
                                    (1 << pHandle->pDmaConfig->etbhalfChannel);
            }
        }
        else if(remWords < 0)
        {
    //        printf(" *** remWords < 0, %d\n",remWords);
            /* Report error if < 0, otherwise nothing to read from ETB */
            return eETB_Overflow;

        }
    }
    else
    {
		//Make sure there are no pending read requests on the TBR slave port
    	//Before issuing an output flush
		retry = 50000;
		do
		{
		   status = *((volatile uint32_t*)TBR_SICTRL(pHandle->id));
		   retry--;
		}while(((status & TBR_READ_REQ_PENDING) != 0) && ( retry != 0 ));

		// Return error if timeout occurs on the above operation
		if (retry == 0)
		{
			RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
		}

    	//Perform an Output flush from the TBR
		// Wait for any previous output flush to complete
		retry = 50000; // Give enough time for a DMA transfer equal to half the size of the TBR
		do
		{
		   status = *(volatile uint32_t*)ETB_FFCR(pHandle->id);
		   retry--;
		}while( ( ( status & TBR_OUTFLUSH_INPROGRESS ) != 0 ) && ( retry != 0 ) );

		// Return error if timeout occurs on the above operation
		if (retry == 0)
		{
			RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
		}

		// Issue an output flush
		status = *(volatile uint32_t*)ETB_FFCR(pHandle->id);
		*(volatile uint32_t*)ETB_FFCR(pHandle->id) = status | TBR_OUTFLUSH_START;

		// Wait for output flush to complete
		retry = 50000;
		do
		{
		   status = *(volatile uint32_t*)ETB_FFCR(pHandle->id);
		   retry--;
		}while(((status & TBR_OUTFLUSH_INPROGRESS) != 0 ) && (retry != 0));

		// Return error if timeout occurs on the above operation
		if (retry == 0)
		{
			RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
		}

		// Wait for final DMA transfer to complete
		retry = 50000; // Give enough time for a DMA transfer equal to half the size of the TBR
		do
		{
		   status = *((volatile uint32_t*)ETB_STS(pHandle->id));
		   retry--;
		}while(((status & TBR_DRAIN_INPROGRESS) != TBR_DRAIN_INPROGRESS) && ( retry != 0 ));

		// Return error if timeout occurs on the above operation
		if (retry == 0)
		{
			RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
		}

		//Make sure there are no pending read requests on the TBR slave port
		retry = 50000;
		do
		{
		   status = *((volatile uint32_t*)TBR_SICTRL(pHandle->id));
		   retry--;
		}while(((status & TBR_READ_REQ_PENDING) != 0) && ( retry != 0 ));

		// Return error if timeout occurs on the above operation
		if (retry == 0)
		{
			RETURN_ETB_CALLBACK(pHandle->id, eETB_Error_Program);
		}

        //Get EDMA buffer information
        if(get_edma_buffer_info(pHandle, pStatus, paramAddress, 0))
        {
            return eETB_Success;
        }
    }
    
#endif //#elif defined(C6657) || defined(C66AK2Hxx)
    
    return eETB_Success;
}

/******************************************************************************/
/*! \copydoc ETB_setDmaStatus
 */
void ETB_setDmaStatus(ETBHandle* pHandle, DMAStatus *pStatus)
{
    if(!pHandle || !pStatus)
    {
        return;
    }
    if(!pHandle->pDmaConfig)
    {
        return;
    }
    pHandle->dmaStatus = *pStatus;
}
#endif //#ifdef DMA_SUPPORT

/******************************************************************************/
/*! \copydoc ETB_getProperties
 */
void ETB_getProperties(ETBProperties *pProperties)
{

#ifdef DMA_SUPPORT

    pProperties->is_dma_supported = 1;

#else

    pProperties->is_dma_supported = 0;

#endif

    return;
}

