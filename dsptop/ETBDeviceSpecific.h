/****************************************************************************
CToolsLib - ETB Library 

Copyright (c) 2009-2012 Texas Instruments Inc. (www.ti.com)
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
#ifndef __ETB_DEVICE_SPECIFIC_H
#define __ETB_DEVICE_SPECIFIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \file ETBDeviceSpecific.h

    This file contains device specific definitions for the ETB library
 */

/************************************************************************** 
   Device directory
**************************************************************************/

/* Keystone 1 devices */
#if (defined(TCI6612) || defined(TCI6614) || defined(TCI6616) || defined(TCI6618) \
|| defined(C6657) || defined(C6670) || defined(C6671) || defined(C6672) || defined(C6674) || defined(C6678)) && !defined(C66x)

#define C66x

#endif

/* For Keystone 2 devices the C66AK2Hxx build works for all devices.
 * The only difference between Keystone 2 devices is the EDMA setup
 * which is performed with a table lookup based on the JTAG ID. (see ETB_config_dma).
 */
#if (defined(TCI6638K2K) || defined(TCI6636K2H) || defined(TCI6630K2L) \
 || defined(AM5K2Exx) || defined(C66AK2Exx) || defined(C66AK2Hxx_CSSTM_ETB)) && !defined(C66AK2Hxx)

#define C66AK2Hxx

#endif

/* For OMAP5 */
#if defined(OMAP54xx)
#define _OMAP54xx
#endif

/* For DRA7x */
#if defined(DRA7xx)
#define _DRA7xx
#endif

/* For TDA3x */
#if defined(TDA3x)
#define _TDA3x
#endif

/**************************************************************************
   Device specific information
   A new device can be supported by adding a device preprocessor
   block specifying base addresses.
**************************************************************************/


#if defined(TCI6486)  
    #define NUM_ETB_INSTANCES 6
    #define SYSETB_PRESENT 0
    #define ARMETB_PRESENT 0

    /* ETB base address for different device types */
    #define _ETB_BaseAddress(n) ( 0x02C40000 + (n<<12) ) /* Base Address for ETB MMRs associated with GEMx */
   
#elif defined(TCI6488) 
    #define NUM_ETB_INSTANCES 3
    #define SYSETB_PRESENT 0
    #define ARMETB_PRESENT 0

    /* ETB base address for different device types */
    #define _ETB_BaseAddress(n) ( 0x02AD0000 + (n<<15) ) /* Base Address for ETB MMRs associated with GEMx */
    
#elif defined(TCI6484) 
    #define NUM_ETB_INSTANCES 1
    #define SYSETB_PRESENT 0
    #define ARMETB_PRESENT 0

    /* ETB base address for different device types */
    #define _ETB_BaseAddress(n) (0x02AD0000) /* Base Address for ETB MMRs associated with GEMx */

    /* PSC MMR address */
    #define PSC_BASE            (0x02AC0000)

#elif defined(C66x)
    #define NUM_ETB_INSTANCES   9 /* For 6616, there are 4 DSP ETB and 1 SYS ETB. For 6678/6608, there are 8 DSP ETB and 1 SYS ETB */
    #define SYSETB_PRESENT 1
    #define ARMETB_PRESENT 0
    #define SYS_ETB_ID  8 /* calculated to get system etb base address using _ETB_BaseAddress(n). For keystone2 devices, SYS_ETB_ID (TBR)
                             is not used to get system etb base address using _ETB_BaseAddress(n).  */
#ifndef __linux
    #define DMA_SUPPORT
#endif

#ifdef __linux
    #define ETB_BaseAddress(n) (0x027D0000 + (n << 16))
    #define _ETB_BaseAddress(n) virtual_ETB_BaseAddress[n]
    #define SIZEOF_ETB_SPACE 4096
#else
    /* Get ETB base address for different cores and system ETB */
    #define _ETB_BaseAddress(n) (0x027D0000 + (n << 16)) /* Base Address for ETB MMRs associated with CorePACx and SYS ETB*/
#endif
    
    /* PSC MMR address */
#ifdef __linux
    #define PSC_BaseAddress     (0x02350000)
    #define PSC_BASE            virtual_PSC_BaseAddress
    #define SIZEOF_PSC_SPACE    4096
#else
    #define PSC_BASE            (0x02350000)
#endif

#elif defined(C66AK2Hxx)

    #define NUM_ETB_INSTANCES   10 /* For 66AK2Hxx, there are 8 DSP ETB, 1 SYS ETB and 1 ARM ETB */
    #define SYSETB_PRESENT 1
    #define ARMETB_PRESENT 1
    #define SYS_ETB_ID  8 /* calculated to get system etb base address using _ETB_BaseAddress(n). For keystone2 devices, SYS_ETB_ID (TBR)
                             is not used to get system etb base address using _ETB_BaseAddress(n).  */
    #define ARM_ETB_ID  9

#ifndef __linux
    #define DMA_SUPPORT
	
	/* Debug SS MIPI STM TBR DMA slave port address */
    #define SYS_TBR_RBD          (0x02850000)

    /* CSSTM TBR DMA slave port address */
    #define ARM_TBR_RBD          (0x027D4000)
#endif

    /* JTAG ID MMR address */
    #define JTAG_ID_REG          (0x2620018)
    #define JTAG_ID_MASK         (0x0FFFFFFF)

    /* ETB MMR address */
#ifdef __linux
    #define ETB_BaseAddress(n) (((n==SYS_ETB_ID) || (n==ARM_ETB_ID))?((n==SYS_ETB_ID)?(0x03019000):(0x03020000)):(0x027D0000 + (n << 16)))
    #define _ETB_BaseAddress(n) virtual_ETB_BaseAddress[n]
    #define SIZEOF_ETB_SPACE 4096
#else
    /* Get ETB base address for different cores and system ETB */
    #define _ETB_BaseAddress(n) (((n==SYS_ETB_ID) || (n==ARM_ETB_ID))?((n==SYS_ETB_ID)?(0x03019000):(0x03020000)):(0x027D0000 + (n << 16))) /* Base Address for ETB MMRs associated with CorePACx and SYS ETB (TBR) */
#endif

    /* PSC MMR address */
#ifdef __linux
    #define PSC_BaseAddress     (0x02350000)
    #define PSC_BASE            virtual_PSC_BaseAddress
    #define SIZEOF_PSC_SPACE    4096
#else
    #define PSC_BASE            (0x02350000)
#endif

#elif defined(TI816x) 
    #define NUM_ETB_INSTANCES   1
    #define SYSETB_PRESENT 1
    #define ARMETB_PRESENT 0
    #define SYS_ETB_ID  0 

    /* ETB base address for different device types */
    // the variable 'n' is used as a dummy and always 'zero' is added to the base address. This is done to remove a compiler warning
    #define _ETB_BaseAddress(n) ( 0x4B162000 + (0 << n) ) /* Base Address for ETB MMRs associated with TI81x - one ETB*/

#elif defined(OMAP3x)//OMAP3
    #error Need to find out ETB base address from device data sheet for this device.

#elif defined(_OMAP) || defined(_OMAP54xx) || defined(_DRA7xx) || defined (_TDA3x)
    #define NUM_ETB_INSTANCES   1
    #define SYSETB_PRESENT      1
    #define ARMETB_PRESENT      0
    #define SYS_ETB_ID          0
	#define SHARED_ETB_DEVICE

#ifdef _OMAP
    /* ETB base address for OMAP devices */
    // the variable 'n' is used as a dummy and always 'zero' is added to the base address. This is done to remove a compiler warning
    #define _ETB_BaseAddress(n) ( 0x54162000 + (0 << n) ) /* Base Address for ETB MMRs - one ETB */
#endif

#if defined(_OMAP54xx) || defined(_DRA7xx) || defined (_TDA3x)
    /* TBR base address for OMAP54xx ES2 devices */
    /* Note that for OMAP5 ES1 the OMAP4430 A9 build will work - _OMAP */
#ifdef __linux
    #define ETB_BaseAddress(n) (0x54167000)
    #define _ETB_BaseAddress(n) virtual_ETB_BaseAddress[n]
    #define SIZEOF_ETB_SPACE 4096
#else
    /* Get ETB base address */
    #define _ETB_BaseAddress(n) (0x54167000) /* Base Address for ETB MMRs - one TBR */
#endif

#endif

#elif defined(TCI6612_CSETB) || defined(TCI6614_CSETB) 
    #define NUM_ETB_INSTANCES   1
    #define SYSETB_PRESENT      1
    #define ARMETB_PRESENT      0
    #define SYS_ETB_ID          0 

    /* ETB base address for TCI6614 device */
    // the variable 'n' is used as a dummy and always 'zero' is added to the base address. This is done to remove a compiler warning
    #define _ETB_BaseAddress(n) ( 0x025A6000 + (0 << n) ) /* Base Address for CS-ETB */

#else
    #error No device type preprocessor defined for the ETBLib
#endif

#ifdef __cplusplus
}
#endif

#endif //__ETB_DEVICE_SPECIFIC_H
