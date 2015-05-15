#ifndef __ETB_INTERFACE_H
#define __ETB_INTERFACE_H

#include <stdint.h> /*ANSI C99 specific type definitions */
/*
 * Embedded Trace Buffer (ETB) API 
 *
 * Copyright (C) 2009-2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
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

#ifdef __cplusplus
extern "C" {
#endif

/*! \file ETBInterface.h
    \version 1.7
    Application Access to Embedded Trace Buffer.

    This module allows users to program ETB debug hardware.
*/
/* The mainpage for doxygen has been added to the end of this file */

/*! \par ETBLIB_MAJOR_VERSION
    ETBLib major revision. This number will be changed for API modifications. 
*/
#define ETBLIB_MAJOR_VERSION    (0x1)

/*! \par ETBLIB_MINOR_VERSION
    ETBLib minor revision. This number will be changed for bug fixes. 
*/
#define ETBLIB_MINOR_VERSION    (0x13)

/*! \par SYS_ETB
    If the ETB is for System Trace (STM), or if a single ETB is shared by multiple cores, use SYS_ETB as a coreID.
*/
#define SYS_ETB 0xFF

/*! \par ARM_ETB
    If the ETB is for ARM A15 Trace (PTM), use ARM_ETB as a coreID.
*/
#define ARM_ETB 0xAA

#if (defined(TCI6612) || defined(TCI6614) || defined(TCI6616) || defined(TCI6618)) && !defined(C6670)
#define C6670
#endif

#if (defined(C6671) || defined(C6672) || defined(C6674)) && !defined(C6678)
#define C6678
#endif


/*! \par eETB_Error
    Common function return error enumeration. 
*/
typedef enum _eETB_Error 
{
    
    eETB_Success = 0,             /*!< Function completed successfully */
    eETB_Error_Bad_Param = -1,    /*!< Error, method parameter error */
    eETB_Error_Program = -2,      /*!< Error, Error programming hardware */
    eETB_Error_Cannot_Own = -3,   /*!< Error, Error programming hardware, ownership cannot be taken */
    eETB_Error_Cannot_Read = -4,  /*!< Error, Cannot read ETB as ETB is not in a readable state */
    eETB_Error_Cannot_Write = -5, /*!< Error, Cannot write ETB as ETB is not in a writabe state */
    eETB_Error_Psc_Enabling = -6, /*!< Error, Error programming hardware, Cannot enable PSC modules for ETB */
    eETB_Overflow = -7,           /*!< Error, Read Overflow - TI Mode only */
    eETB_Underflow = -8           /*!< Error, Read Underflow - TI Mode only */
} eETB_Error ;


/*! \par ETB_errorCallback
    The callback function is called with the function's exit status.  Note that this function is not exported
    by the interface, but is used as a parameter for calls. 
    
    \param[in] eETB_Error error returned by calling routine.
    \return void 

    \par Details:
    \details   
    This is a user provided callback normally used to centralize error handling and error is desired to be handled in a 
    callback routine provided by the caller instead a return error code. 
*/
typedef void(*ETB_errorCallback)(eETB_Error);


/*! \par eETB_Mode
    Type of ETB mode. This is used to define ETB mode during an ETB_open call.
    This mode should be very carefully selected as per the actual ETB on chip.
*/
typedef enum _eETB_Mode
{
    eETB_Circular = 0,                  /*!< Circular mode for ETB11, CSETB, TI ETB. Only circular buffer mode and can't be read while capturing*/
    eETB_TI_Mode = 1,                   /*!< For TI-ETB implemetation, this is TI ETB mode (simultaneneous read/write) with a circular buffer mode
                                             For Keystone2 SYS ETB and CSSTM ETB, provides circular buffer mode with no support for simultaneneous read/write of ETB buffer*/
    eETB_Stop_Buffer = 2,               /*!< Stop on buffer full mode*/
    eETB_TI_Mode_AND_Stop_Buffer = 3    /*!< For TI-ETB implemetation, this is TI ETB mode (simultaneneous read/write) with stop on buffer full mode
                                        For Keystone2 SYS ETB and CSSTM ETB, provides stop on buffer full mode with no support for simultaneneous read/write of ETB buffer */
} eETB_Mode ;

/*! \par eDMA_Mode
    Type of DMA mode. This is used to define the DMA mode during a DMA configuraion.
*/
typedef enum _eDMA_Mode
{
    eDMA_Circular = 0,                  /*!< Circular mode */
    eDMA_Stop_Buffer = 1                /*!< Stop on buffer full mode */
} eDMA_Mode ;

/*! \par eCIC_Select
    The Chip-level Interrupt Controller number values.
*/
typedef enum _eCIC_Select
{
    eCIC_0,
    eCIC_1,
    eCIC_2,
    eCIC_3
} eCIC_Select;

/*! \par DMAConfig
    \brief DMA configuration structure
*/
typedef struct _DMAConfig
{
#if defined(C6670) || defined(C6678)
    
    uint32_t cc;           /*!< EDMA3 channel controller number. Only applicable for C6670 and C6678 devices. */
    uint16_t clrChannel;   /*!< 1st DMA channel number for DMA draining (needs to
                                 be able to access CPINTC). Application must have
                                 configured the channel to PaRAM mapping for this
                                 channel before using this API. Only applicable for C6670 and C6678 devices.
                            */
    uint16_t etbChannel;   /*!< 2nd DMA channel number for DMA draining (needs to
                                 be able to access the ETB). Application must have
                                 configured the channel to PaRAM mapping for this
                                 channel before using this API. Only applicable for C6670 and C6678 devices.
                            */
    eCIC_Select cic;       /*!< External chip level interrupt controller INTCx,
                                 also called Chip Interrupt Controller CICx.
                                 Selection from the eCIC_Select enumeration.
                                 Only applicable for C6670 and C6678 devices.
                            */
                            
#endif
                            
    uint16_t linkparam[3]; /*!< 3 additional parameter RAM entry numbers (used for
                                 linking by the library). Thses must be different
                                 PaRAM entries than thos mapped to channel 1 & 2.
                            */
    uint32_t dbufAddress;  /*!< DMA Drain buffer address */
    uint32_t dbufWords;    /*!< DMA Drain buffer size in 32-bit words */
    eDMA_Mode mode;        /*!< DMA Drain buffer mode (only eDMA_Circular and
                                 eDMA_Stop_Buffer are valid values)
                            */
} DMAConfig;

/*! \par DMAStatus
    \brief DMA status structure
    \details
    This structure is populated in the ETB_flush_dma function to provide the
    address for the beginning of the circular buffer and the number of words
    that have been transferred into it. The buffer address and size that are
    provided in the DMA configuration structure is duplicated here to provide
    all of the information required to manage reading the circular buffer.
*/
typedef struct _DMAStatus
{
    uint32_t startAddr; /*!< Current starting lcoation address in DMA drain
                              buffer.
                        */
    uint32_t availableWords; /*!< Total number of 32-bit words that have been written
                                   into the DMA drain buffer.
                             */
    uint32_t isWrapped;    /*!< DMA Drain buffer is full and has started writing
                                 over previously written words.
                           */
    uint32_t dbufAddress;  /*!< DMA Drain buffer address */
    uint32_t dbufWords;    /*!< DMA Drain buffer size in 32-bit words */
    uint32_t flushRequired; /*!< Only allow flushing ETB once related to DMA */
} DMAStatus;

/*! \par ETBHandle
    ETB Handle object. This is an incomplete structure, thus making the actual implementation
    private to the ETBLib.
*/

typedef struct _ETBHandle_t ETBHandle;

/*! \par ETBHandle_Pntr
    Pointer to a ETB Handle object
*/

typedef ETBHandle * ETBHandle_Pntr;

/*! \par ETBStatus
    \brief ETB status structure definition.
*/
typedef struct _ETBStatus
{
    uint8_t canRead; /*!< ETB can be read*/
    uint8_t isWrapped; /*!< ETB is wrapped */
    uint32_t availableWords; /*!< ETB has the available words to be read */
    uint32_t ETB_TraceCaptureEn; /*!< ETB trace capture is enabled or not */
    uint32_t overflow; /*!< ETB overflow occurred (only used if trying to read
                            while writting the ETB in non-DMA mode) */                        
} ETBStatus;

/*! \par ETBCapability
    \brief ETB status structure definition.
*/
typedef struct _ETBProperties
{
    uint8_t is_dma_supported; /*!< ETB-EDMA extension supported or not*/

}ETBProperties;

/*! \par ETB_open
    \brief Open and initialize ETB.

    \param[in] pErrCallBack is called if not NULL and this function returns any eETB_Error value other than eETB_Success.
    \param[in] mode is the mode in which ETB should be used. Most commonly it is eETB_Circular.
    \param[in] coreID core ID (0,1,2....) for a device with homogeneous DSPs or CPUs and each core has a dedicated ETB. For example, TCI6488 utilizes 0,1 or 2 for this parameter. For device with a single core/ETB combination, 0 is used.
    If the ETB is for System Trace (STM),  or if a single ETB is shared by multiple cores, use SYS_ETB as a coreID.
    \param[out] ppHandle is pointer to a ETBhandle pointer, the pointer is allocated by the ETBLib so caller should not allocate the pointer. This should be passed back to "ETB_close()" call, once done.
    \param[out] pETBSizeInWords contains size of ETB buffer in 32 bit words, if successfully opened. 
    \return eETB_Error. 
    
    \par Details:
    \details  This function must be the first ETBLib function called to initialize ETB module access. All other ETBLib fiunctions will return eETB_Error_Program if ETB_open has not been called.
    If the initialization process is successful a handle pointer that is core specific (not library instance specific) is returned.

    \note
    Note: ETBLib provides a static allocation of handle memory in the ETBLib_ExtMem data section, allowing the handle memory to be shared between cores.
    The user must link ETBLib_ExtMem to shared memory to enable handle sharing between cores.
    In this case ETB_open would be called once from the core A, and then ETB_gethandle called from core B to get the shared handle.
    All other ETBLib functions can then be called from core B.
    ETB_open must be called with a NULL ETB_errorCallback, unless the callback function is shared between cores.
    No synchronization between cores is provided within ETBLib.

*/
eETB_Error  ETB_open(ETB_errorCallback pErrCallBack, eETB_Mode mode, uint8_t coreID, ETBHandle** ppHandle, uint32_t* pETBSizeInWords);

/*! \par ETB_gethandle
    \brief Get the handle of an ETBLib opened for the specified coreID.

    \param[in] coreID core ID (0,1,2....) for a device with homogeneous DSPs or CPUs and each core has a dedicated ETB. For example, TCI6488 utilizes 0,1 or 2 for this parameter. For device with a single core/ETB combination, 0 is used.
    If the ETB is for System Trace (STM),  or if a single ETB is shared by multiple cores, use SYS_ETB as a coreID.
    \param[out] ppHandle is pointer to a ETBhandle pointer.
    \return eETB_Error.

    \par Details:
    \details If ETBLib is already opened for the specified coreID, the handle pointer is returned.
    The return value is eETB_Error_Program, if ETBLib has not been opened for the specified core.
*/
eETB_Error  ETB_gethandle(uint8_t coreID, ETBHandle** ppHandle);

/*! \par ETB_enable
    Enable ETB to start capturing data. 

    \param[in] pETBHandle ETB Handle pointer.
    \param[in] triggerCount is number of words written to ETB RAM following a trigger event. This is only used for ARM. Use 0 for DSP.
    \return eETB_Error. 
    
    \par Details:
    \details This function enables ETB. As soon as ETB is enabled, it starts capturing trace data.
    The ETB should be enabled after trace export, clocks and data trace formatter (if applicable) has been programmed.
*/
eETB_Error  ETB_enable(ETBHandle* pETBHandle, uint32_t triggerCount);

/*! \par ETB_disable
    Disable ETB to stop capturing data. 

    \param[in] pETBHandle ETB Handle pointer.
    \return eETB_Error. 
    
    \par Details:
    \details  This function disables ETB. As soon as ETB is disabled, it stops capturing trace data.
    A CoreSight ETB is ready to be read once it is disabled. 
    A TI ETB in TI_ETB mode does not need to be disabled before reading out the ETB contents.
*/
eETB_Error  ETB_disable(ETBHandle* pETBHandle);

/*! \par ETB_status
    Function to check the ETB status to check if it is ready to be read.

    \param[in] pETBHandle ETB Handle pointer.
    \param[out] status contains ETB status parameters. 
    \return eETB_Error. 
    
    \par Details:
    \details  This function provides information ETB state.
    Before making a call to ETB_read , it is expected to call this method to get the state of the ETB.
*/
eETB_Error  ETB_status(ETBHandle* pETBHandle, ETBStatus* status);

/*! \par ETB_read
    Function to read out ETB RAM contents.

    \param[in] pETBHandle ETB Handle pointer.
    \param[in] pBuffer an allocated buffer pointer passed in that would contain ETB buffer on return. 
    \param[in] bufferLength is the size of the allocated buffer in 32 bit long words unit. 
    \param[in] startWord is the index (0 based) of the ETB word to start reading. ETB_canread() provides number of available words. Use 0 for reading from a valid start of buffer.
    \param[in] requestSize is the requested number of words to be read from the ETB.
    \param[out] pRetSize contains the actual number of read words and retured as part of pBuffer.
    \return eETB_Error. 
    
    \par Details:
    \details This function reads ETB contents, if the ETB can be read.
    Caller of this function is responsible to allocate and deallocate the buffer.
    If the request size is more than available ETB words and the buffer size is at least the request size, request sise is returned.
    If the buffer is NULL or allocated size is less than the requested words, an error is returned.
    
    \note
    If using the DMA drain buffer, ETB_flush_dma <b>MUST</b> be called before
    using this read function.
    
*/
eETB_Error  ETB_read(ETBHandle* pETBHandle, uint32_t *pBuffer, uint32_t bufferLength, uint32_t startWord, uint32_t requestSize, uint32_t* pRetSize);

/*! \par ETB_close
    Function to close the ETB and relese ETB handle pointer.

    \param[in] pETBHandle ETB Handle pointer.
    \return eETB_Error 
    
    \par Details:
    \details This function should be the last call made once your are done with the ETB.
    After closing th ETB, ETB_open call is requuired before ETB can be used again.
*/
eETB_Error  ETB_close(ETBHandle* pETBHandle);

/*! \par ETB_flush
    Flush the ETB. 

    \param[in] pHandle ETB Handle pointer.
    \return eETB_Error. 
    
    \par Details:
    \details This function flushes the ETB input buffer and ADTF. If you have temporarily stopped trace export
    with TEND, the flush will provide the end of the current trace packet, avoiding the "insufficient
    data" error when decoding.

    If DSP Core-ETB is configured in EDMA mode, the ADTF is both flushed and stopped by ETB_flush().
    If DSP Core-ETB is configured in non-EDMA mode, only the ADTF is flushed by ETB_flush(). In this case 
    the ADTF is stopped in the ETB_disable function.
    If System ETB is configured either in EDMA or non-EDMA mode, STM data is flushed to the ETB by ETB_flush().
    
    This function is only valid if the library has been opened in TI mode.
    Note that if you are not in TI mode the ETB_disable function performs the flush operation.
    
    Note: When the ETB is configured in EDMA mode, ETB_flush() must be called prior to ETB_flush_dma(). 
    
*/
eETB_Error  ETB_flush(ETBHandle* pHandle);

/*! \par ETB_config_dma
    Configure the DMA used to interface with the ETB. 

    \param[in] pHandle
    \param[in] pConfig
    \return eETB_Error. 

    \par Details:
    \details This function is passed a configuration structure that is used to setup the
    EDMA3 to transfer data written into the ETB to a location in memory. The
    ETB half-full and full interrupts are used as system events to start each
    transaction. The interrupts are routed from the ETB through the chip-level
    INTCx (interrupt controller) to an input event in the EDMA3. Two DMA channels
    are required, one for clearing the INTCx system interrupt status register,
    and the second for transferring data from the ETB to the drain buffer in
    memory.
    
    ETB_open must be called before this configuration function with the option
    eETB_TI_Mode. Any other option will cause this function to return in error.
    
    \note The application is responsible for all DMA channel and parameter
    RAM mapping and configuration.

*/

eETB_Error  ETB_config_dma(ETBHandle* pHandle, const DMAConfig *pConfig);

/*! \par ETB_flush_dma
    Configure a DMA transfer to flush remaining words in ETB to memory buffer.

    \param[in] pHandle
    \param[in] pStatus
    \return eETB_Error. 

    \par Details:
    \details This function is used to copy any remaining information in the ETB that
    has not reached the half-full or full mark in the buffer. Status information
    is populated in the DMAStatus structure that can be used to manage collecting
    information from the drain buffer in memory.

    The DMA drain functions will only use a multiple of half of the the ETB
    size. If the number of words that is passed as part of the DMA configuration
    is not a multiple of half the ETB size, the DMA status value in the handle
    will get set to the number of words that are actually being used.
    

    \note
    The application must have stopped all tracing and ETB_flush function must be
    called before this function.
    
*/
eETB_Error  ETB_flush_dma(ETBHandle* pHandle, DMAStatus *pStatus);

/*! \par ETB_setDmaStatus
    Set the DMAStatus structure used by the DMA library functions.

    \param[in] pHandle
    \param[in] pStatus

    \par Details:
    \details When using the DMA drain functionality with multiple DMA buffers, this
    function is required to set the correct status parameters before making
    dependent API calls such as ETB_read.

*/
void ETB_setDmaStatus(ETBHandle* pHandle, DMAStatus *pStatus);

/*! \par ETB_getProperties
    Set the properties(capabilties supported) by the ETB.

    \param[in] pProperties

    \par Details:
    \details This API can be used by the application to determine the capabilities
    supported by ETB of a particular device. For example: The Application
    can determine whether ETB-DMA extension is supported or not.
*/
void ETB_getProperties(ETBProperties *pProperties);

#ifdef __cplusplus
}
#endif

#endif //__ETB_INTERFACE_H

/******************************************************************************/
/* *****  DOXYGEN  ***** */
/******************************************************************************/
/*! \mainpage
\par Introduction
The ETB Library provides an application interface to program and drain the
Embedded Trace Buffer (ETB). There are two kinds of ETB in TI devices, CSETB and
TIETB. The ETBLib contains CCSv5 projects in the ETBLib/project directory
specifying the supported devices and rules to pickup specific files and flags
for different devices. \n\n
For C66xx devices using the TIETB in TI mode (enabling the ETB to be read while
enabled and actively getting written) there is also an option to use the EDMA3
to drain the ETB to specified memory buffer.

\par ETBLib_ExtMem data section
There are two TI-ETBLib features that are enabled by linking the ETBLib_ExtMem
data section to shared/non-cacheable memory.
- ETBLib handle sharing: This feature allows the ETB of core A to be initialized with a call
to ETB_open from core A, and then additional ETBLib calls can be made from core B to access
the core A ETB. See ETB_open() and ETB_gethandle() for details.
- CCS ETBLib data visualization: If you are using ETBLib in your application, but are debugging
with CCS, this features allows visualization of trace data exported by ETBLib to external memory.
A set of ETBLib symbols are allocated to the ETBLib_ExtMem section that are utilized by CCS
to extract the trace data from memory. To take advantage of this feature, in CCSv5.4 and later,
you simply need to change the "Transport Type" of a "Custom PC Trace" configuration
(Tools->Hardware Trace Analyzer->Custom PC Trace) to "ETB Remote-memory".

\par EDMA3 Drain Buffer
The DMA drain buffer is used to increase the amount of storage used for
trace data beyond the size of the ETB. The DMA uses the half-full and full
interrupt from the ETB as a trigger to start a transaction that will read half
of the ETB for each interrupt. For all Keystone1 devices except C6657, The ETB 
interrupts must get routed through the chip-level interrupt controller (INTCx) 
to interface with the EDMA3 controller. Where as in case of all keystone2 and 
C6657, the ETB half and full events are directly connected to the EDMA3 controller.
\n\n
\subpage edma3_page
\n
\note If the Drain Buffer is located in MSMC or DDR3 memory, it should be put in
a non-cacheable region.
\n

\par Devices supported by the library are:
 - AM437x
 - C6A816x, also referenced as TI816x (Netra)
 - C66xx (C6670, TCI6614/16/18)
 - C6678
 - OMAP-A9
 - OMAP-A15
 - TCI6484
 - TCI6488
 - C6657
 - C66AK2Hxx (Keystone 2)

\par API Functions:
 - #ETB_open
 - #ETB_gethandle
 - #ETB_enable
 - #ETB_disable
 - #ETB_status
 - #ETB_read
 - #ETB_close
 - #ETB_flush
 - #ETB_config_dma
 - #ETB_flush_dma
 - #ETB_setDmaStatus
 - #ETB_getProperties

*/
