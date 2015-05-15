/* =============================================================================
 *
 *            TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION
 *
 *  Property of Texas Instruments
 *  For Unrestricted Internal Use Only 
 *  Unauthorized reproduction and/or distribution is strictly
 *  prohibited.
 *  This product is protected under copyright law and trade secret law  
 *  as an unpublished work.
 *  Created 2011, (C) Copyright 2011 Texas Instruments.  All rights 
 *  reserved.
 */
/**
 *  @Component    STP2_DEC 
 * 
 *  @Filename     stpv2_dec.h
 *
 *  @Description  Declaration of functions used in STPV2_DEC component
 *
 *//*======================================================================== */

#ifndef __STP2_DEC_H
#define __STP2_DEC_H

/* =============================================================================
 * INCLUDE FILES (only if necessary)
 * =============================================================================
 */


#include "env_headers.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* =============================================================================
 * EXPORTED DEFINITIONS
 * =============================================================================
 */

/*-------------------------------------------------------------------------*//**
 * @DEFINITION   <Identifier>
 *
 * @BRIEF        <Brief description of macro>   
 *
 * @DESCRIPTION  <Detailed description of macro>
 *
 *//*------------------------------------------------------------------------ */

/* =============================================================================
 * EXPORTED TYPES
 * =============================================================================
 */
 
 typedef struct stp2_dec_device *stp2_dec_device_ptr;

/*-------------------------------------------------------------------------*//**
 * @TYPE         return_code_stp2_dec
 *
 * @BRIEF        Error code which reports 0 if return of function is OK, else reports
 *               an error.
 *  
 *               STP2_DEC_SUCCESS 
 *               
 *
 * @DESCRIPTION  <Detailed description of type>
 *
 *//*------------------------------------------------------------------------ */

enum return_code_stp2_dec 
{
   stp2_dec_success = 0,
   stp2_dec_err_null_parameter,         
   stp2_dec_err_internal_decoder_error,
   stp2_dec_err_currently_unsupported_timestamp_format,
   stp2_dec_err_lost_sync,
   stp2_dec_err_memory_alloc_failure, 
};


/* 16 possible values of STP2 version nibble payload */
enum stp2_version
{
    STP2_VERSION_V1_A = 0x0,  /* STP 1.0 timestamp */
    STP2_VERSION_V1_B,        /* STP 1.0 timestamp */
    STP2_VERSION_V2_NATDELTA, /* Natural binary delta timestamp */
    STP2_VERSION_V2_NAT,      /* Natural binary timestamp */
    STP2_VERSION_V2_GRAY,     /* Reflected binary absolute timestamp */
    STP2_VERSION_RESERVED0,   /* Illegal value */
    STP2_VERSION_RESERVED1,   /* Illegal value */
    STP2_VERSION_RESERVED2,   /* Illegal value */
    STP2_VERSION_RESERVED3,   /* Illegal value */
    STP2_VERSION_RESERVED4,   /* Illegal value */
    STP2_VERSION_RESERVED5,   /* Illegal value */
    STP2_VERSION_RESERVED6,   /* Illegal value */
    STP2_VERSION_RESERVED7,   /* Illegal value */
    STP2_VERSION_RESERVED8,   /* Illegal value */
    STP2_VERSION_RESERVED9,   /* Illegal value */
    STP2_VERSION_RESERVED10   /* Illegal value */
};



/* Only nibble of single nibble packet type is defined */
enum stp2_sng_nibble_packet_type
{
    STP2_PKT_TYPE_NONE      = 0x0,  /* Payload is none  */
    STP2_PKT_TYPE_M8        = 0x1,  /* Payload = M[7:0]    -- current_master[7:0] = 0, current_channel = 0; */
    STP2_PKT_TYPE_MERR      = 0x2,  /* Payload = E[7:0]    -- current_channel= 0 */
    STP2_PKT_TYPE_C8        = 0x3,  /* Payload = C[7:0] */
    STP2_PKT_TYPE_D8        = 0x4,  /* Payload = D[7:0] */
    STP2_PKT_TYPE_D16       = 0x5,  /* Payload = D[15:0] */
    STP2_PKT_TYPE_D32       = 0x6,  /* Payload = D[31:0] */
    STP2_PKT_TYPE_D64       = 0x7,  /* Payload = D[63:0] */
    STP2_PKT_TYPE_D8MTS     = 0x8,  /* Payload = D[7:0] + TS */
    STP2_PKT_TYPE_D16MTS    = 0x9,  /* Payload = D[15:0] + TS */
    STP2_PKT_TYPE_D32MTS    = 0xA,  /* Payload = D[31:0] + TS */
    STP2_PKT_TYPE_D64MTS    = 0xB,  /* Payload = D[63:0] + TS */
    STP2_PKT_TYPE_D4        = 0xC,  /* Payload = D[3:0] */
    STP2_PKT_TYPE_D4MTS     = 0xD,  /* Payload = D[3:0] + TS */
    STP2_PKT_TYPE_FLAG_TS   = 0xE,  /* Payload = TS */
    STP2_PKT_TYPE_EXT_OPC   = 0xF   /* Payload is none -- First nibble of multi-nibble opcode */
};



/* Second nibble of two nibble packet type is defined */
enum stp2_two_nibble_packet_type
{
    STP2_SEC_NIB_PKT_TYPE_EXT_OPC   = 0x0, /* Payload is none -- Second nibble of multi-nibble opcode */ 
    STP2_SEC_NIB_PKT_TYPE_M16       = 0x1, /* Payload = M[15:0]         --- current_master  = 0; current_channel = 0 */
    STP2_SEC_NIB_PKT_TYPE_GERR      = 0x2, /* Payload = E[7:0]          --- current_master  = 0; current_channel = 0 */
    STP2_SEC_NIB_PKT_TYPE_C16       = 0x3, /* Payload = C[15:0]         --- current_channel = 0; */
    STP2_SEC_NIB_PKT_TYPE_D8TS      = 0x4, /* Payload = D[7:0] + TS */
    STP2_SEC_NIB_PKT_TYPE_D16TS     = 0x5, /* Payload = D[15:0] + TS */
    STP2_SEC_NIB_PKT_TYPE_D32TS     = 0x6, /* Payload = D[31:0] + TS */
    STP2_SEC_NIB_PKT_TYPE_D64TS     = 0x7, /* Payload = D[63:0] + TS */
    STP2_SEC_NIB_PKT_TYPE_D8M       = 0x8, /* Payload = D[7:0] */
    STP2_SEC_NIB_PKT_TYPE_D16M      = 0x9, /* Payload = D[15:0] */
    STP2_SEC_NIB_PKT_TYPE_D32M      = 0xA, /* Payload = D[31:0] */
    STP2_SEC_NIB_PKT_TYPE_D64M      = 0xB, /* Payload = D[63:0] */
    STP2_SEC_NIB_PKT_TYPE_D4TS      = 0xC, /* Payload = D[3:0] + TS */
    STP2_SEC_NIB_PKT_TYPE_D4M       = 0xD, /* Payload = D[3:0] */
    STP2_SEC_NIB_PKT_TYPE_RESERVED0 = 0xE, /* Reserved opcode  */
    STP2_SEC_NIB_PKT_TYPE_ASYNC_SRT = 0xF  /* Reserved opcode but could be 2nd nibble of an async sequence  */
};


/* Third nibble of three nibble packet type is defined */
enum stp2_three_nibble_packet_type
{
    STP2_THR_NIB_PKT_TYPE_VERSION   = 0x0, /* Payload = D[3:0]  -- current_master = 0; current_channel = 0; */
    STP2_THR_NIB_PKT_TYPE_NULL_TS   = 0x1, /* Payload = TS  */
    STP2_THR_NIB_PKT_TYPE_USER      = 0x2, /* Payload = user data = nibble[0] (length in nibbles -1), up to 16 nibbles of data */
    STP2_THR_NIB_PKT_TYPE_TSG       = 0x2, /* Payload = TSG[7:4] - TS granularity for OMAP5ES2 MIPI  */
    STP2_THR_NIB_PKT_TYPE_USER_TS   = 0x3, /* Payload = user data + TS */
    STP2_THR_NIB_PKT_TYPE_TIME      = 0x4, /* Payload = nibble[0] (clock_ID) + nibble[1] (version) + time */
    STP2_THR_NIB_PKT_TYPE_TIME_TS   = 0x5, /* Payload = nibble[0] (clock ID) + nibble[1] (version) + time + TS */
    STP2_THR_NIB_PKT_TYPE_TRIG      = 0x6, /* Payload = ID[7:0] */
    STP2_THR_NIB_PKT_TYPE_TRIG_TS   = 0x7, /* Payload = ID[7:0] + TS */
    STP2_THR_NIB_PKT_TYPE_FREQ      = 0x8, /* Payload = FREQ[31:0] */
    STP2_THR_NIB_PKT_TYPE_FREQ_TS   = 0x9, /* Payload = FREQ[31:0] + TS */
    STP2_THR_NIB_PKT_TYPE_XSYNC     = 0xA, /* Payload = ID[7:0] */
    STP2_THR_NIB_PKT_TYPE_XSYNC_TS  = 0xB, /* Payload = ID[7:0] + TS */
    STP2_THR_NIB_PKT_TYPE_RESERVED0 = 0xC, /* Reserved opcode */
    STP2_THR_NIB_PKT_TYPE_RESERVED1 = 0xD, /* Reserved opcode */
    STP2_THR_NIB_PKT_TYPE_RESERVED2 = 0xE, /* Reserved opcode */
    STP2_THR_NIB_PKT_TYPE_RESERVED3 = 0xF, /* Reserved opcode */
};


union stp2_pkt_header_type
{
    enum stp2_sng_nibble_packet_type    sng_nibble;
    enum stp2_two_nibble_packet_type    two_nibble;
    enum stp2_three_nibble_packet_type  three_nibble;
};

/*-------------------------------------------------------------------------*//**
 * @TYPE         stp2_packet
 *
 * @BRIEF        Contents of an STP2.0 packet created by the STP2.0 Decoder component
 *
 * @DESCRIPTION  <Detailed description of type>
 *
 *//*------------------------------------------------------------------------ */

struct stp2_packet
{
	u16           					master_id; 	 		/* Master ID associated to STP opcode  */
	u16            					channel_id;    		/* Channel ID associated to STP opcode */       
	union stp2_pkt_header_type      opcode;     		/* The STP2 opcode type */
    u8             					opcodeLength;       /* The number of valid nibbles in the opcode */
	u8            					payloadLength;      /* Number of valid data nibbles */
	u64            					payload;            /* Number of nibbles of payload, up to D64 being supported */
	                                     				/* payload[0] is LSB nibble; payload[15] is MSB nibble */
    u64            					timestamp;          /* Current 64-bit absolute (binary) time associated with this packet */
};

/*-------------------------------------------------------------------------*//**
 * @TYPE         stp2_nibble_pos
 *
 * @BRIEF        Enumerated type for the 2 nibble positions in a byte.
 *               STP2.0 being a nibble protocol but the input stream to decode
 *               is a stream of bytes. The current nibble position to decode next
 *               is maintained.
 *
 * @DESCRIPTION  <Detailed description of type>
 *
 *//*------------------------------------------------------------------------ */

enum stp2_nibble_pos
{
    STP2_NIBBLE_POS_LOWER=0,
    STP2_NIBBLE_POS_UPPER,
    STP2_NIBBLE_POS_RESERVED
};

/* =============================================================================
 * EXPORTED VARIABLES
 * =============================================================================
 */

/*-------------------------------------------------------------------------*//**
 * @VARIABLE     <Identifier>
 *
 * @BRIEF        <type> <Identifier> - <Brief description of variable>   
 *
 * @DESCRIPTION  <Detailed description of variable>
 *
 *//*------------------------------------------------------------------------ */


/* =============================================================================
 * EXPORTED FUNCTIONS
 * =============================================================================
 */

/* ------------------------------------------------------------------------*//**
 * @FUNCTION      stp2_dec_open
 *
 * @BRIEF         STP2_DEC component open
 *
 * @param[in,out] stp2_dec_device  - STP2_DEC device
 *
 * @RETURNS       sts_err_unknown_err - Error returned if device is not defined
 *                sts_success         - Correct execution
 *
 * @DESCRIPTION   STP2_DEC component initialization
 *
 *//*------------------------------------------------------------------------ */
extern enum return_code_stp2_dec stp2_dec_open(stp2_dec_device_ptr*);



/* ------------------------------------------------------------------------*//**
 * @FUNCTION      stp2_dec_close
 *
 * @BRIEF         STP2_DEC component closure
 *
 * @param[in,out] stp2_dec_device  - STP2_DEC device
 *
 * @RETURNS       sts_err_unknown_err - Error returned if device is not defined
 *                sts_success         - Correct execution
 *
 * @DESCRIPTION   STP2_DEC component closure
 *
 *//*------------------------------------------------------------------------ */
extern enum return_code_stp2_dec stp2_dec_close(struct stp2_dec_device* );


/* ------------------------------------------------------------------------*//**
 * @FUNCTION      stp2_dec_decode
 *
 * @BRIEF         Primary API Call for STP2.0 Decoder
 *
 * @param[in] stp2_dec_device       - STP2_DEC device [IN]
 * @param[in] const u8*             - A byte array to be decoded allocated by caller [IN]
 * @param[in] u32                   - The number of bytes allocated in the byte array [IN] (minimum 1...2^32-1)
 * @param[in] u32                   - The position of the next byte to be decoded, starts at 0 [IN]
 * @param[in] stp2_nibble_pos       - The position of the next nibble in next byte to be decoded [IN]
 * @param[in] u32                   - The maximum number of STP2.0 packets to be decoded [IN]
 * @param[in]                       - True when sync sequence detection is needed and False otherwise [IN]
 * @param[out] stp2_packet*         - The decoded STP2.0 packet array returned by API [OUT]
 * @param[out] u32*                 - The number of packets decoded which is less than or equal to 
 *                                    maximum number requested. [OUT]
 * @param[out] u32*                 - The position of the next byte to be decoded [OUT]
 *                                    This should be beyond the caller allocated buffer if all bytes have been
 *                                    exhausted. [OUT]
 * @param[out] stp2_nibble_pos*      - The position of the next nibble in the next byte to be decoded [OUT]
 *                                     This is a don't care if the previous parameter indicates all bytes in the
 *                                     caller's buffer have been exhaused [OUT]
 * @param[out] BOOL*                 - Whether all bytes in the caller's buffer have been decoded or not [OUT]
 * @param[out] BOOL*                 - Whether the decoding ends on a message boundary or not [OUT]
 *                
 *
 * @RETURNS       stp2_dec_reserved_value - ERROR since 0xF value is reserved and cannot be used
 *                stp2_dec_success        - Correct execution
 *
 * @DESCRIPTION   This API decodes a caller's allocated STP2.0 byte buffer array and returns
 *                decoded samples in the caller's allocated Decoder buffer. The decoded samples
 *                are returned as an array of STP2.0 packet structures.
 *
 *//*------------------------------------------------------------------------ */
extern enum return_code_stp2_dec stp2_dec_decode 
                                          (struct stp2_dec_device*,
                                           const u8*,
                                           u32,
                                           u32,   
                                           enum stp2_nibble_pos,
                                           u32,
										   BOOL,
                                           struct stp2_packet*,
                                           u32*, 
                                           u32*, 
                                           enum stp2_nibble_pos*,
										   BOOL*,
										   BOOL*
                                           );
#ifdef __cplusplus
}
#endif
#endif	/* __STP2_DEC_H */
/* EOF */

