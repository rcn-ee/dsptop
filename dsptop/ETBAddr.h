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
#ifndef __ETB_ADDR_H
#define __ETB_ADDR_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \file ETBAddr.h

    This file contains the ETB and TI Data Trace Formatter addresses.
    The "n" parameter used for the address is a CPU enumeration for homogeneous multi-core devices.
    For example, TCI6488 has n values as 0,1,3 for the three DSPs. 
    If a device does not have such multiple cores and associated ETBs, n is unused and should be 0.
    where ETB is being programmed and accessed.
 */

#if defined(ETB_HOSTED)

#include "ETBLib_Hosted.h"

#else

//Include device specific ETB definitions
#include "ETBDeviceSpecific.h"

#endif

/* Keystone PSC registers */
#define PSC_PDSTAT(n)       (PSC_BASE + 0x200 + (4*n)) /* memory mapped address for Power Domain Status Register */
#define PSC_PDCTL(n)        (PSC_BASE + 0x300 + (4*n)) /* memory mapped address for Power Domain Control Register */
#define PSC_MDCTL(n)        (PSC_BASE + 0xA00 + (4*n)) /* memory mapped address for Module Control Register for clock, reset and EMU behavior control */
#define PSC_MDSTAT(n)       (PSC_BASE+ 0x800 + (4*n)) /* memory mapped address for Module status register */
#define PSC_PTCMD           (PSC_BASE + 0x120) /* memory mapped address for transition command register */
#define PSC_PTSTAT          (PSC_BASE + 0x128)  /* memory mapped address for transition command status register */

/* Registers common for both TI-ETB and TBR implementations*/

/* ETB RAM Depth Register RDP */
/* TBR RAM Size Register  */
#define ETB_RDP(n)          (_ETB_BaseAddress(n) + 0x004)
/* ETB Status Register STS */
/* TBR Status Register */
#define ETB_STS(n)          (_ETB_BaseAddress(n) + 0x00C)
/* ETB/TBR RAM Read Data Register RRD */
#define ETB_RRD(n)          (_ETB_BaseAddress(n) + 0x010)
/* ETB/TBR RAM Read Pointer Register RRP */
#define ETB_RRP(n)          (_ETB_BaseAddress(n) + 0x014)
/* ETB/TBR RAM Write Pointer Register RWP */
#define ETB_RWP(n)          (_ETB_BaseAddress(n) + 0x018)
/* ETB/TBR Trigger counter register */
#define ETB_TRIG(n)         (_ETB_BaseAddress(n) + 0x01C)
/* ETB/TBR Control Register CTL */
#define ETB_CTL(n)          (_ETB_BaseAddress(n) + 0x020)
/* ETB/TBR RAM Write Data Register RWD */
#define ETB_RWD(n)          (_ETB_BaseAddressn(n)+ 0x024)
/* ETB Formatter and Flush Status Register FFSR */
/* TBR Operation Status Register OPSTAT */
#define ETB_FFSR(n)         (_ETB_BaseAddress(n) + 0x300)
/* ETB Formatter and Flush Control Register FFCR */
/* TBR Operations Control Register OPCTRL*/
#define ETB_FFCR(n)         (_ETB_BaseAddress(n) + 0x304)
/* ETB/TBR Lock Access Register */
#define ETB_LOCK(n)         (_ETB_BaseAddress(n) + 0xFB0)
/* ETB/TBR Lock Status Register */
#define ETB_LOCK_STATUS(n)  (_ETB_BaseAddress(n) + 0xFB4)
/* ETB/TBR device ID Register */
#define ETB_DEVID(n)        (_ETB_BaseAddress(n) + 0xFC8)
/* ETB/TBR Peripheral ID 0 Register */
#define ETB_PERIPH_ID0(n)   (_ETB_BaseAddress(n) + 0xFE0)
/* ETB/TBR Peripheral ID 1 Register */
#define ETB_PERIPH_ID1(n)   (_ETB_BaseAddress(n) + 0xFE4)

/* Registers specific to TI-ETB implementation*/
#define ETB_WIDTH(n)        (_ETB_BaseAddress(n) + 0x008) /* ETB RAM Width Register STS */
#define ETB_RBD(n)          (_ETB_BaseAddress(n) + 0xA00) /* ETB RAM burst read Register */
#define ETB_TI_CTL(n)       (_ETB_BaseAddress(n) + 0xE20) /* ETB TI Control Register */
#define ETB_IRST(n)         (_ETB_BaseAddress(n) + 0xE00) /* ETB TI Interrupt Raw Status Register */
#define ETB_ICST(n)         (_ETB_BaseAddress(n) + 0xE04) /* ETB TI Interrupt Raw Status Register */
#define ETB_IER(n)          (_ETB_BaseAddress(n) + 0xE0C) /* ETB TI Interrupt Enable Register */
#define ETB_IECST(n)        (_ETB_BaseAddress(n) + 0xE10) /* Clear interrupt enable bits */

/* Registers specific to TBR implementation */
#define TBR_FIFOSZ(n)          (_ETB_BaseAddress(n) + 0x008) /*TBR Output FIFO Size Register */
#define TBR_OUTLVL(n)          (_ETB_BaseAddress(n) + 0x100) /*Output FIFO Trigger Level Register */
#define TBR_SICTRL(n)          (_ETB_BaseAddress(n) + 0x104) /*TBR System Interface Control */
#define TBR_IDPERIOD(n)        (_ETB_BaseAddress(n) + 0x108) /*ID Repeat Period Register */
#define TBR_SEQCNTL(n)         (_ETB_BaseAddress(n) + 0x10C) /*Message Sequence Insertion Control */
#define TBR_EOI(n)             (_ETB_BaseAddress(n) + 0x120) /*TBR EOI register */
#define TBR_IRQSTATUS_RAW(n)   (_ETB_BaseAddress(n) + 0x124) /*TBR IRQ Status (Raw) register */
#define TBR_IRQSTATUS(n)       (_ETB_BaseAddress(n) + 0x128) /*TBR IRQ Status register */
#define TBR_IRQENABLE_SET(n)   (_ETB_BaseAddress(n) + 0x12C) /*TBR IRQ Enable set register */
#define TBR_IRQENABLE_CLR(n)   (_ETB_BaseAddress(n) + 0x130) /*TBR IRQ Enable clear register */
#define TBR_CLAIMSET(n)        (_ETB_BaseAddress(n) + 0xFA0) /*Claim Tag Set Register */
#define TBR_CLAIMCLR(n)        (_ETB_BaseAddress(n) + 0xFA4) /*Claim Tag Clear Register */
#define TBR_AUTHSTAT(n)        (_ETB_BaseAddress(n) + 0xFB8) /*Authorization Status Register */


/* ETB enable bit */
#define ETB_ENABLE      0x00000001
/* ETB Status Register Bit definitions */
#define ETB_STS_ACQCOMP  0x00000004 /* bit 2: 1=acquisition complete */
#define ETB_STS_FULL     0x00000001 /* bit 0: 1=RAM full */
/* ETB Formatter and Flush Status Register bits */
#define ETB_FLUSH_INPROGRESS 0x00000001 /* bit 0: 1 = flush in progress */
#define ETB_STOPFORMATTER_INPROGRESS 0x00000010 /* bit 1: 1 = formatter stop in progress */
/* ETB unlock value */
#define ETB_UNLOCK_VAL   0xC5ACCE55 /* Value to unlock ETB for register accesses */
/* ETB lock bits */
#define LOCK_STATUS_IMP_BIT		(1<<0)
#define LOCK_STATUS_STAT_BIT	(1<<1)
/* ETB modes */
#define TI_ETB_CIRCULARMODE_BIT	(0x1<<1)
#define TI_ETB_TI_MODE          (0x1<<0)

#define TBR_BRIDGE_MODE         (0x1<<1)
#define TBR_BUFFER_MODE         (0xFFFFFFFD)

#define TI_ETB_IRST_UNDERFLOW (1 << 3)
#define TI_ETB_IRST_OVERFLOW  (1 << 2)
#define TI_ETB_IRST_FULL      (1 << 1)
#define TI_ETB_IRST_HALF_FULL (1 << 0)

#define TBR_IRST_AQCMP      (1 << 1)
#define TBR_IRST_DAV        (1 << 0)
#define TBR_STP_FULL        (1 << 15)

#define TBR_TWP_DISABLE (0xFFFFFFFE)
#define TBR_TWP_ENABLE  (0x1)

#define TBR_TWP_IDPERIOD (0x8)

#define TBR_TWP_SEQATBID (0x6F << 16)
#define TBR_TWP_SEQPERIOD (0x10)

/* TBR Operations Control Register (OPCTRL)bits */
#define TBR_OUTFLUSH_INPROGRESS (1<<16) /* bit 16: 1 = output flush in progress */
#define TBR_OUTFLUSH_START (1<<16) /* set bit 16 = 1 : output flush in started */

/* TBR Status Register(STAT) bits */
#define TBR_DRAIN_INPROGRESS (1<<4) /* bit 4: 1 = DMA drain in progress */

/* TBR System interface control (SICTRL) register */
#define TBR_READ_REQ_PENDING (1<<1)

#define ARM_TBR_NUMBLOCK (0xF)
#define ARM_TBR_BLOCKSZ (0x1F)
#define SYS_TBR_NUMBLOCK (0xF)
#define SYS_TBR_BLOCKSZ (0x3F)

/* TI-ETB and TBR device identifiers */

/* Note, for a TBR the Device ID register holds the major/minor revision of the TBR,
 * and there are devices with TBR revisions 0x11, 0x12 and 0x13.
 * For a TI-ETB the Device ID is a fixed value (0x20) and someday we could have a 0x20
 * TBR revision. A better solution to distinguish between ETB and TBR is to use the
 * Peripheral ID 0/1 register.
 */
#define TBR_DEVICE_TYPE (0xEDF)
#define ETB_PERIPH_ID0_MASK (0x000000FF)
#define ETB_PERIPH_ID1_MASK (0x0000000F)


/* ETB Maximum Burst Size value */
#define ETB_BURST_SIZE  0x400

#ifdef __cplusplus
}
#endif

#endif //__ETB_ADDR_H
