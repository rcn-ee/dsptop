/*
 * stm.h
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
#ifndef STM_H
#define STM_H

#if defined(C66AK2Hxx)

#define STM_XPORT_BASE_ADDR1      0x20000000
#define STM_XPORT_BASE_ADDR2      0x20080000
#define STM_CHAN_RESOLUTION       0x1000
#define STM_CONFIG_BASE_ADDR      0x03018000

#define STM_SW_MASTER_MAP         0x0C080400
#define STM_SW_MASTER_MASK        0x03030303
#define STM_HW_MASTER_MAP         0x80808080

#define MIPI_STM_TBR_SIZE         0x8000        /* 32 KB - only required for STM 2.0 Modules */

#elif defined(DRA7xx)

#define STM_XPORT_BASE_ADDR1      0x54000000
#define STM_XPORT_BASE_ADDR2      0x54080000
#define STM_CHAN_RESOLUTION       0x1000
#define STM_CONFIG_BASE_ADDR      0x54161000

#define STM_SW_MASTER_MAP         0x34200400
#define STM_SW_MASTER_MASK        0x03030303
#define STM_HW_MASTER_MAP         0x747C7860

#define MIPI_STM_TBR_SIZE         0x8000        /* 32 KB - only required for STM 2.0 Modules */

#define CSTF_DSS_BASE_ADDR        0x54164000

#else

#error "Supported device not defined"

#endif

/* Definitions for CoreSight Trace Funnel */
#ifdef CSTF_DSS_BASE_ADDR

#define CSTF_CNTLREG_BLKSIZE        4096
#define CSTF_UNLOCK_OFFSET          0xFB0
#define CSTF_DSS_LOCK(ba)           ((volatile uint32_t *)((uint32_t)(ba) + CSTF_UNLOCK_OFFSET))
#define CSTF_DSS_CTRL(ba)           ((volatile uint32_t *)((uint32_t)(ba)))

#define CSTF_DSS_CTRL_CTSTM         (1 << 7)

#endif

/* There are 256 STM channels. Since the linux DebugSS driver will only allow a memory space to be mapped
 * to one process, the STM space is being split in two. One space for OpenCL and the second space for dsptop.
 * So ulm_config will map 128 channels to STM_XPORT_BASE_ADDR1 (XPORT_ONLY build) for OpenCL and 128 channels
 * to STM_XPORT_BASE_ADDR2 for dsptop */
#define STM_CH_NUM_MIPI              128

#define STM_CNTLREG_BLKSIZE          4096
#define CS_UNLOCK_VALUE              0xC5ACCE55

#define STM_CS_REGOFF_Lock           0xFB0
#define STM_CS_REGOFF_LockStatus     0xFB4

#define STM_CS_LOCK(ba)              ((volatile uint32_t *)((uint32_t)(ba) + STM_CS_REGOFF_Lock))
#define STM_CS_LOCKSTATUS(ba)        ((volatile uint32_t *)((uint32_t)(ba) + STM_CS_REGOFF_LockStatus))

#define STM_FLUSH_RETRY       1024
#define STM_CLAIM_RETRY       100

/* STM ATB IDs */
#define STM_ATB_ID_MIPI              0x40
#define STM_ATB_ID_ARM2_0            0x41

/* MIPI STM Control registers offsets */
#define STM_MIPI_REGOFF_Id           0x000
#define STM_MIPI_REGOFF_SysConfig    0x010
#define STM_MIPI_REGOFF_SysStatus    0x014
#define STM_MIPI_REGOFF_SWMstCntl_0  0x024
#define STM_MIPI_REGOFF_SWMstCntl_1  0x028
#define STM_MIPI_REGOFF_SWMstCntl_2  0x02C
#define STM_MIPI_REGOFF_SWMstCntl_3  0x030
#define STM_MIPI_REGOFF_SWMstCntl_4  0x034
#define STM_MIPI_REGOFF_HWMstCntl    0x038
#define STM_MIPI_REGOFF_PTIConfig    0x03C
#define STM_MIPI_REGOFF_PTICntDown   0x040
#define STM_MIPI_REGOFF_ATBConfig    0x044
#define STM_MIPI_REGOFF_ATBSyncCnt   0x048
#define STM_MIPI_REGOFF_ATBHeadPtr   0x048     /* Same offset as ATBSyncPtr so we must know the revision of the STM unit. */
#define STM_MIPI_REGOFF_ATBid        0x04C


#define STM_MIPI_ID_REG(ba)          ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_Id))
#define STM_MIPI_SYSCNFG(ba)         ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SysConfig))
#define STM_MIPI_SYSSTAT(ba)         ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SysStatus))
#define STM_MIPI_SWMSTCNTL0(ba)      ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SWMstCntl_0))
#define STM_MIPI_SWMSTCNTL1(ba)      ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SWMstCntl_1))
#define STM_MIPI_SWMSTCNTL2(ba)      ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SWMstCntl_2))
#define STM_MIPI_SWMSTCNTL3(ba)      ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SWMstCntl_3))
#define STM_MIPI_SWMSTCNTL4(ba)      ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_SWMstCntl_4))
#define STM_MIPI_HWMSTCNTL(ba)       ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_HWMstCntl))
#define STM_MIPI_PTICNFG(ba)         ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_PTIConfig))
#define STM_MIPI_PTICNTDOWN(ba)      ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_PTICntDown))
#define STM_MIPI_ATBCNFG(ba)         ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_ATBConfig))
#define STM_MIPI_ATBHEADPTR(ba)	     ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_ATBHeadPtr))
#define STM_MIPI_ATBSYNCCNT(ba)	     ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_ATBSyncCnt))
#define STM_MIPI_ATBID(ba)	         ((volatile uint32_t *)((uint32_t)(ba) + STM_MIPI_REGOFF_ATBid))

/* MIPI STM specific definitions */
#define STM_MIPI_NUM_CHANNELS 256
#define STM_FIFO_EMPTY        (1<<8)
#define STM_MAJOR_MASK        (7<<8)
#define STM_MAJOR_STM2_0      0x400

/* STM_MIPI_SWMSTCNTL0 definitions */
#define OWNERSHIP_MASK     (3<<30)
#define MOD_AVAILABLE      (0<<30)
#define MOD_CLAIMED        (1<<30)
#define MOD_ENABLED        (2<<30)
#define DEBUG_OVERRIDE     (1<<29)
#define CURRENT_OWNER_MASK (1<<28)
#define CURRENT_DEBUG_OWNS (0<<28)
#define CURRENT_APP_OWNS   (1<<28)
#define STM_TRACE_EN       (1<<16)

typedef enum
{
    eBUFSIZE_0,                         /*!< STM data will be drained into a fixed buffer */
	eBUFSIZE_4K,                        /*!< STM data will be drained into a 4K circular buffer */
	eBUFSIZE_8K,                        /*!< STM data will be drained into a 8K circular buffer */
	eBUFSIZE_16K,                       /*!< STM data will be drained into a 16K circular buffer */
	eBUFSIZE_32K,                       /*!< STM data will be drained into a 32K circular buffer */
	eBUFSIZE_64K,                       /*!< STM data will be drained into a 64K circular buffer */
	eBUFSIZE_128K,                      /*!< STM data will be drained into a 128K circular buffer */
	eBUFSIZE_256K                       /*!< STM data will be drained into a 256K circular buffer */
} eSTM_ExportBufSize;

/* OST Headers - just in case */
#define OST_VERSION				(1 << 24)						/* Restrictions: value must be between 1 and 15 */
#define OST_ENTITY_ID			(1 << 16)
#define OST_PROTOCOL_PRINTF		(0x80 << 8)						/* STMXport_printf (STM messages pass %s pointer) */
#define OST_PROTOCOL_BYTESTREAM	(0x81 << 8)	                    /* STMXport_putMsg */
#define OST_PROTOCOL_MSGSTREAM	(0x82 << 8)	                    /* STMXport_putBuf */
#define OST_PROTOCOL_INTERNALMSG (0x83 << 8)                    /* STMXport_IntMsg (Internal Message) */
#define OST_PROTOCOL_PRINTF_UNOPTIMIZED (0x84 << 8)             /* STMXport_printf (STM messages pass %s byte stream) */
#define OST_SHORT_HEADER_LENGTH_LIMIT	(255)

#endif /* STM_H */
