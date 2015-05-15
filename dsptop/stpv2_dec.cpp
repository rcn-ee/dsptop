/* $Header: <fileName>.c	<lastVersionNumber>    	<lastEditionDate> 
 * =============================================================================
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
 *  @Filename     STP2_DEC.cpp
 *
 *  @Description  STP 2.0 Protocol Decoder 
 *
 *//*======================================================================== */


/* =============================================================================
 * STANDARD INCLUDE FILES
 * =============================================================================
 */
#include <stdlib.h>
#include <stdio.h>

/* =============================================================================
 * PROJECT SPECIFIC INCLUDE FILES
 * =============================================================================
 */
#include "stpv2_dec.h"

/* =============================================================================
 * The original definition for nibble conversion is:
 * NIBBLE(byte, pos)( (pos) == STP2_NIBBLE_POS_LOWER ? (byte&0xF) : ((byte&0xF0)>>4) )
 *
 * Now the data from the data source saves each nibble to a byte, no conversion is reqired.
 * =============================================================================
 */
#define NIBBLE(byte, pos) ( byte=byte )

#define STP2_ASYNC_PATTERN_F_LENGTH  (21)  /* Number of 0xF nibbles in STP2 ASYNC pattern */
#define STP2_ASYNC_PATTERN_LENGTH_MAX (22) /* Number of 0xF nibbles plus a 0x0            */

/* TIME_TS has the max number of nibbles in an STP2 message
 * 3 nibbles for header + 19 nibbles in payload + 17 nibbles in timestamp */
#define STP2_MAX_MESSAGE_LENGTH    (39)


/* =============================================================================
 * GLOBAL VARIABLES DECLARATIONS
 * =============================================================================
 */

/* =============================================================================
 * LOCAL TYPES AND DEFINITIONS
 * =============================================================================
 */
 
enum pkt_hdr_dec_info
{
	PKT_HDR_DEC_SUCCESS = 0,
	PKT_HDR_DEC_ASYNC_START,
	PKT_HDR_DEC_INSUFFICIENT_BYTES,
	PKT_HDR_DEC_RESERVED
};



struct stp2_packet_attributes
{
    BOOL payload_length_known; /* Payload length is always known (excluding ts) except for USER/TIME packets which have variable length, ignoring RESERVED s  */
    BOOL timestamp_present;    /* Whether timestamp is present or not */
    u8   payload_length;       /* Payload length in nibbles */
};


struct timestamp_nibbles
{
    u8   ts_nibbles[16];
    u8   ts_nibble_length;
};


/*-------------------------------------------------------------------------*//**
 * @TYPE         stp2_device
 *
 * @BRIEF        Contents of an STP2.0 Decoder handle 
 *
 * @DESCRIPTION  <Detailed description of type>
 *
 *//*------------------------------------------------------------------------ */
struct stp2_dec_device
{
    u16                 current_master_id;   /* Current master  ID */
    u16                 current_channel_id;  /* Current channel ID */
    u8                  stp_version_number;  /* Version 0x3 = STPv2NAT; 0x4 = STPv2GRAY */
    BOOL                sync_sequence_found; /* FALSE = sync sequence never found; TRUE = sync sequence found */
    enum stp2_version   version;             /* STP2.0 Version Packet payload which affects how timestamps are decoded */
    u64                 current_time;        /* Current time */
    u32                 timestamp_frequency; /* 32-bit value indicating the freq of clock generating timestamp */
	u8                  partial_message_cache[STP2_MAX_MESSAGE_LENGTH]; /* cache for array of nibbles in a partial message */
	u8                  partial_message_cache_length;                   /* Number of nibbles in the message cache */  
	u8                  current_TS_granularity; /* current TS granularity used for OMAP5ES2 MIPI to caculate delta TS. Default is 0. */
};



/* =============================================================================
 * LOCAL VARIABLES DECLARATIONS
 * =============================================================================
 */

// Sync sequence is 21 0xF plus 0x0
const static u8 async_pattern[STP2_ASYNC_PATTERN_LENGTH_MAX] = 
            { 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x0 };


const static struct stp2_packet_attributes stp_three_nibble_packet_attributes[] = 
{ 
    { True,    False, 0x1  },  /* VERSION    */
    { True,    True,  0x0  },  /* NULL_TS    */
    { False,   False, 0x0  },  /* USER       */
    { False,   True,  0x0  },  /* USER_TS    */
    { False,   False, 0x0  },  /* TIME       */
    { False,   True,  0x0  },  /* TIME_TS    */
    { True,    False, 0x2  },  /* TRIG       */
    { True,    True,  0x2  },  /* TRIG_TS    */
    { True,    False, 0x8  },  /* FREQ       */
    { True,    True,  0x8  },  /* FREQ_TS    */
    { True,    False, 0x2  },  /* XSYNC      */
    { True,    True,  0x2  },  /* XSYNC_TS   */
    { False,   False, 0x0  },  /* RESERVED0  */
    { False,   False, 0x0  },  /* RESERVED1  */
    { False,   False, 0x0  },  /* RESERVED2  */
    { False,   False, 0x0  }   /* RESERVED3  */
};


const static struct stp2_packet_attributes stp_sng_packet_attributes[] = 
{  
    { True,  False, 0x0  },  /* NULL packet */
    { True,  False, 0x2  },  /* M8   */
    { True,  False, 0x2  },  /* MERR */
    { True,  False, 0x2  },  /* C8 */
    { True,  False, 0x2  },  /* D8 */
    { True,  False, 0x4  },  /* D16 */
    { True,  False, 0x8  },  /* D32 */
    { True,  False, 0x10 },  /* D64 */
    { True,  True,  0x2  },  /* D8MTS */
    { True,  True,  0x4  },  /* D16MTS */
    { True,  True,  0x8  },  /* D32MTS */
    { True,  True,  0x10 },  /* D64MTS */
    { True,  False, 0x1  },  /* D4 */
    { True,  True,  0x1  },  /* D4MTS */
    { True,  True,  0x0  },  /* FLAG_TS */
    { False, False, 0x0  }   /* EXT_OPC */
};


const static struct stp2_packet_attributes stp_two_nibble_packet_attributes[] = 
{ 
    { False, False,  0x0  },  /* EXT_OPC  */
    { True,  False,  0x4  },  /* M16    */
    { True,  False,  0x2  },  /* GERR   */
    { True,  False,  0x4  },  /* C16    */
    { True,  True,   0x2  },  /* D8TS   */
    { True,  True,   0x4  },  /* D16TS  */
    { True,  True,   0x8  },  /* D32TS  */
    { True,  True,   0x10 },  /* D64TS  */
    { True,  False,  0x2  },  /* D8M    */
    { True,  False,  0x4  },  /* D16M   */
    { True,  False,  0x8  },  /* D32M   */
    { True,  False,  0x10 },  /* D64M   */
    { True,  False,  0x1  },  /* D4M    */
    { True,  True,   0x1  },  /* D4MTS  */
    { False, False,  0x0  },  /* RES    */
    { False, False,  0x0  }   /* RES    */
};


/* =============================================================================
 * LOCAL FUNCTIONS PROTOTYPES
 * =============================================================================
 */
static enum return_code_stp2_dec update_absolute_timestamp 
                             (const u64                        current_time,   /* This is current absolute time (binary) */
							  const u64                   current_granularity, /* This is the current cranularity    */
                              const struct timestamp_nibbles  *raw_timestamp,  /* This is up to 16 nibbles of raw timestamp and len information */
                              enum stp2_version                version,        /* This is the version information */
                                    u64                       *new_time);      /* This API returns the new absolute time into caller's buffer */


static enum return_code_stp2_dec decode_packet_timestamp 
                             (const u8                    *list_of_bytes,                /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                          number_of_bytes,              /* Total number of bytes in the caller's buffer */
                              u32                          byte_pos,                     /* The position of the next byte to be decoded */
                              enum  stp2_nibble_pos        nibble_pos,                   /* The position of the current nibble to be decoded */
                              enum stp2_version            version,                      /* The current STP2.0 version number format of the timestamp to be extracted */
                              u32                         *new_byte_pos,                 /* This API returns the position of next byte to be decoded after timestamp extraction  */
                              enum stp2_nibble_pos        *new_nibble_pos,               /* This API returns the position of the next nibble to be decoded after timestamp extraction */
							  BOOL                        *timestamp_decoded,            /* This API returns whether the timestamp has been decoded or not */					
                              u8                          *number_of_timestamp_nibbles,  /* This API returns the number of timestamp nibbles found */
                              u8                          *timestamp_nibble_values );    /* This API puts the timestamp nibbles into caller's buffer */         


static enum return_code_stp2_dec decode_packet_payload 
                            ( const u8                    *list_of_bytes,                /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                          number_of_bytes,              /* Total number of bytes in the caller's buffer */
                              u32                          byte_pos,                     /* The position of the next byte to be decoded */
                              enum  stp2_nibble_pos        nibble_pos,                   /* The position of the current nibble to be decoded */
                              union stp2_pkt_header_type   pkt_header_type,              /* The type of packet header detected  */
                              u8                           pkt_header_num_nibbles,       /* The number of nibbles in the packet header */
                              enum stp2_version            version,                      /* The current STP2.0 version number */
                              u32                         *new_byte_pos,                 /* This API returns the position of next byte to be decoded after packet header extraction  */
                              enum stp2_nibble_pos        *new_nibble_pos,               /* This API returns the position of the next nibble to be decoded after packet header extraction */
							  BOOL                        *pkt_payload_decoded,          /* This parameter returns if payload is decoded or not */					
                              u8                          *number_of_payload_nibbles,    /* This API returns the number of payload nibbles detected */
                              u8                          *payload_data,                 /* This API puts the payload nibbles into caller's buffer */
                              u8                          *number_of_timestamp_nibbles,  /* This API returns the number of timestamp nibbles found */
                              u8                          *timestamp_nibble_values );     /* This API puts the timestamp nibbles into caller's buffer */


static enum return_code_stp2_dec decode_packet_header 
                            ( const u8                    *list_of_bytes,           /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                          number_of_bytes,         /* Total number of bytes in the caller's buffer */
                              u32                          byte_pos,                /* The position of the next byte to be decoded */
                              enum stp2_nibble_pos         nibble_pos,              /* The position of the current nibble to be decoded */
                              u32                         *new_byte_pos,            /* This API returns the position of next byte to be decoded after packet header extraction  */
                              enum stp2_nibble_pos        *new_nibble_pos,          /* This API returns the position of the next nibble to be decoded after packet header extraction */
							  enum pkt_hdr_dec_info       *pkt_hdr_dec_info,        /* This API returns whether pkt hdr is decoded or if bytes in input buffer are exhausted or not or if start of an async sequence has been found */					
                              union stp2_pkt_header_type  *pkt_header_type,         /* Returns the type of packet found */
                              u8                          *pkt_num_nibbles );       /* Returns the number of nibbles found in packet */



static enum return_code_stp2_dec search_async_sequence 
                            ( const u8              *list_of_bytes,    /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                    number_of_bytes,  /* Total number of bytes in the caller's buffer */
                              BOOL                   first_async_search, /* FALSE = first async pattern search and True = Subsequent async pattern search */
                              u32                    byte_pos,         /* The position of the next byte to be decoded */
                              enum stp2_nibble_pos   nibble_pos,       /* FALSE = next nibble to be decoded is bits 3:0; TRUE = bits 7:4 */
                              u32                   *new_byte_pos,     /* This API returns the position of next byte to be decoded after async detection and extraction */
                              enum stp2_nibble_pos  *new_nibble_pos,   /* This API returns the position of the next nibble to be decoded after async detection and extraction */
							  BOOL                  *partial_async,    /* This API returns if a partial async was found or not */
							  BOOL                  *async_found);     /* This parameter returns whether an async sequence is found or not */


u64 bin_to_gray( const u64 nlast_timestamp );
u64 gray_to_bin( const u64 nnew_gray );

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     stp2_dec_open  
 *
 * @BRIEF        Brief description of function purpose
 *
 * @DESCRIPTION  Detailed description of function purpose
 *
 * @RETURNS      Description of return value
 *
 * @NOTES        This section is optional, delete if unused. Should contain  
 *               anything else important about the function, e.g correct usage
 * 
 *//*------------------------------------------------------------------------ */
enum return_code_stp2_dec stp2_dec_open(struct stp2_dec_device **device)
{
	struct stp2_dec_device *temp_ptr;
  
    /* Check for input parameter errors */
    if (device == NULL)
    {
        return stp2_dec_err_null_parameter;
    }
	
	/* Assign a default value to the return parameter */
	*device = NULL;
	
	
	temp_ptr =  (struct stp2_dec_device*) malloc (sizeof(struct stp2_dec_device));

	if (temp_ptr == NULL)
	{
		return stp2_dec_err_memory_alloc_failure;
	}
	
	*device = temp_ptr; 
	
	/* Assign default values to handle parameters */
    temp_ptr->current_master_id    = 0;
    temp_ptr->current_channel_id   = 0;
    temp_ptr->stp_version_number   = STP2_VERSION_RESERVED10;
    temp_ptr->sync_sequence_found  = False;
    temp_ptr->current_TS_granularity   = 0;
    
    /* Only 48-bit timestamps supported in OMAP-5 & 6 */
    temp_ptr->current_time      = 0x0000ULL; /* Unknown */
    
    temp_ptr->timestamp_frequency  = 0xFFFFFFFF; /* Unknown */

    return stp2_dec_success;
}

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     Function Name  
 *
 * @BRIEF        Brief description of function purpose
 *
 * @DESCRIPTION  Detailed description of function purpose
 *
 * @RETURNS      Description of return value
 *
 * @NOTES        This section is optional, delete if unused. Should contain  
 *               anything else important about the function, e.g correct usage
 * 
 *//*------------------------------------------------------------------------ */
enum return_code_stp2_dec stp2_dec_close(struct stp2_dec_device *device)
{

	/* Check for input parameter error */
    if (device == NULL)
    {
        return stp2_dec_err_null_parameter;
    }

    device->current_master_id         = 0;
    device->current_channel_id        = 0;
    device->stp_version_number        = STP2_VERSION_RESERVED10;
    device->sync_sequence_found       = False;
    device->current_time              = 0x0000FFFFFFFFFFFFULL; /* Unknown */    
    device->timestamp_frequency       = 0xFFFFFFFF;
    device->current_TS_granularity   = 0;
	
	/* De-allocate the memory associated with the handle */
	free (device);

    return stp2_dec_success;
}

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     Function Name  
 *
 * @BRIEF        Brief description of function purpose
 *
 * @DESCRIPTION  Detailed description of function purpose
 *
 * @RETURNS      Description of return value
 *
 * @NOTES        This section is optional, delete if unused. Should contain  
 *               anything else important about the function, e.g correct usage
 * 
 *//*------------------------------------------------------------------------ */
enum return_code_stp2_dec stp2_dec_decode 
                                          (struct stp2_dec_device *device,
                                           const u8               *list_of_bytes, 
                                           u32                     length_of_bytes,
                                           u32                     byte_pos,    
                                           enum stp2_nibble_pos    nibble_pos,
                                           u32                     maximum_number_of_packets_to_decode, 
										   BOOL                    sync_detect,        
                                           struct stp2_packet     *packets_decoded,              /* Returned by API */
                                           u32                    *number_of_packets_decoded,    /* Returned by API */
                                           u32                    *next_byte_pos,                /* Returned by API */
                                           enum stp2_nibble_pos   *next_nibble_pos,              /* Returned by API */
										   BOOL                   *end_of_input_buffer,          /* Returned by API */
										   BOOL                   *partial_message               /* Returned by API */
                                           )
										   
{                                   

	enum return_code_stp2_dec   return_value                    = stp2_dec_success;
                     u32        current_byte_pos                = 0;
    enum stp2_nibble_pos        current_nibble_pos              = STP2_NIBBLE_POS_RESERVED;
                     u32        new_byte_pos                    = 0;
    enum stp2_nibble_pos        new_nibble_pos                  = STP2_NIBBLE_POS_RESERVED;
	union stp2_pkt_header_type  pkt_hdr_type;
                                pkt_hdr_type.sng_nibble         = STP2_PKT_TYPE_NONE;
                    u8          pkt_header_nibbles              = 0;
                    u8          number_of_payload_nibbles       = 0;
                    u8          payload_data[16];
                    u32         pkt_decoded                     = 0;
                    BOOL        pkt_type_to_be_stored           = False;
    struct timestamp_nibbles    ts_raw_data;
                    BOOL        version_pkt_need_to_be_detected = False;
                    u64         new_time                        = 0;
                    BOOL        first_sync_sequence             = False;
                    u32         start_byte_pointer              = 0;
    enum stp2_nibble_pos        start_nibble_pointer            = STP2_NIBBLE_POS_RESERVED;
                    u32         index                           = 0;
					BOOL        all_bytes_decoded               = False;
					BOOL        partial_message_end             = True;   /* Negated if necessary */
					BOOL        decode_error                    = False;	
    enum pkt_hdr_dec_info       pkt_hdr_info                    = PKT_HDR_DEC_RESERVED;	

    				u64         nibble_data                     = 0;
    				
    				//u8 nibble_data                     = 0;
					u8          nibble_index                    = 0;
					BOOL        partial_async_seq               = False;
					BOOL        async_seq_found                 = False;	
					BOOL        pkt_payload_decoded             = False; 

    /* Error check */
    if ((device                       == NULL) ||
        (list_of_bytes                == NULL) ||
        (packets_decoded              == NULL) ||
        (number_of_packets_decoded    == NULL) ||
        (next_byte_pos                == NULL) ||
        (next_nibble_pos              == NULL) ||
		(end_of_input_buffer          == NULL) ||
		(partial_message              == NULL) ||
		(length_of_bytes              == 0 ) )
    {
        return stp2_dec_err_null_parameter;
    }
	

    /* Return parameters in case of error being returned */
    *number_of_packets_decoded      = 0;
    *next_byte_pos                  = byte_pos;
    *next_nibble_pos                = nibble_pos;
	*end_of_input_buffer            = False;
	*partial_message                = True;  /* Negated if necessary */
	
	
	if (current_byte_pos >= length_of_bytes)
	{
		*end_of_input_buffer  = True;
		
		return stp2_dec_success;
	}

    /* Start decoding packets */
    pkt_decoded = 0;

    /* Initialize the byte and nibble pointers */
    current_byte_pos   = byte_pos;
    current_nibble_pos = nibble_pos;
	
	/* Caller requests whether sync is needed or not */
	if (sync_detect)
	{
		device->sync_sequence_found = False;
	}
	else
	{
		device->sync_sequence_found = True;
	}

    /* A sync sequence is required to be detected if it was not previously found */
    if (!device->sync_sequence_found)
    {
        first_sync_sequence = True;
    }

	/* Decode packets until either the decoded packet buffer is full or until there are no more bytes to be decoded or 
	 * until there is a decode error.
	 */
	 
	decode_error      = False;
	all_bytes_decoded = False;
	
	
	
    while ( (all_bytes_decoded == False)  && (pkt_decoded < maximum_number_of_packets_to_decode) && 
    		(decode_error == False) /*&& (current_byte_pos<length_of_bytes)*/ )
    {
        /* start byte and nibble pointers in case the current pointers have to be unwound */
        /* Current pointers have to be unwound in case of error and in 
         * case a partial async sequence has been detected due to a header 
		 */
        start_byte_pointer   =  current_byte_pos;
        start_nibble_pointer =  current_nibble_pos;

        /* Local Parameter */
        ts_raw_data.ts_nibble_length = 0;

        /* Indicates whether stp packet is to be stored in the caller's buffer */
        /* By default True, must be negated if desired */
        pkt_type_to_be_stored = True;
        pkt_header_nibbles    = 0;
	
        /* Search for the async sequence if it needs to be */
        if (device->sync_sequence_found == False)
        {
		    /* A version pkt must be seen immediately after an async packet has been found */
            version_pkt_need_to_be_detected = True;
			
            return_value = search_async_sequence 
                                    ( list_of_bytes, 
                                      length_of_bytes,
                                      first_sync_sequence,
                                      current_byte_pos,
                                      current_nibble_pos,
                                     &new_byte_pos,
                                     &new_nibble_pos,
									 &partial_async_seq,
									 &async_seq_found);
									
			if (return_value == stp2_dec_success)
			{   
				if (async_seq_found == True)
				{
					device->sync_sequence_found = True;
					
					if (first_sync_sequence)
					{
						first_sync_sequence = False;
					}
					
					current_byte_pos   = new_byte_pos;
            		current_nibble_pos = new_nibble_pos;
				}
				else if ((partial_async_seq) && (first_sync_sequence))
				{
					all_bytes_decoded  = True;
					decode_error       = True;
					return_value       = stp2_dec_err_lost_sync;
				}
				else if ((partial_async_seq) && (!first_sync_sequence))
				{
					all_bytes_decoded  = True;
					decode_error       = True;
				}
				else
				{
					return_value       = stp2_dec_err_lost_sync;
				} 
			}
			else
			{
				decode_error = True;
			}
		}
		// we should have found the async sequence
		if (device->sync_sequence_found == True)
		{
			if (current_byte_pos != length_of_bytes)
			{
        			/* Get the next packet header */
					return_value = decode_packet_header
                            					( list_of_bytes,
                              					  length_of_bytes,
                              					  current_byte_pos,
                              					  current_nibble_pos,
                             					 &new_byte_pos,
                             					 &new_nibble_pos,
												 &pkt_hdr_info,
                             					 &pkt_hdr_type,
                             					 &pkt_header_nibbles );
												 												 
					if ((return_value == stp2_dec_success) && ( pkt_hdr_info == PKT_HDR_DEC_SUCCESS))
					{							
            			current_byte_pos    = new_byte_pos;
            			current_nibble_pos  = new_nibble_pos;

            			if (version_pkt_need_to_be_detected)
            			{
                			/* Check a version packet has been found */
                			if ((pkt_header_nibbles == 3) && (pkt_hdr_type.three_nibble == STP2_THR_NIB_PKT_TYPE_VERSION))
                 			{
                    			version_pkt_need_to_be_detected = False;

                			}
						}
						
						/* Only continue with decoding if the version packet no longer needs to be detected */
						if (!version_pkt_need_to_be_detected)
						{
	        				/* Update the master & channel id fields in the packet */
            				packets_decoded[pkt_decoded].master_id   = device->current_master_id;
            				packets_decoded[pkt_decoded].channel_id  = device->current_channel_id; 

            				packets_decoded[pkt_decoded].opcodeLength  = pkt_header_nibbles;

            				if (pkt_header_nibbles == 1)
            				{
                				packets_decoded[pkt_decoded].opcode.sng_nibble = pkt_hdr_type.sng_nibble;
            				}
            				else if (pkt_header_nibbles == 2)
            				{
                				packets_decoded[pkt_decoded].opcode.two_nibble = pkt_hdr_type.two_nibble;
            				}
            				else if (pkt_header_nibbles == 3)
            				{
                				packets_decoded[pkt_decoded].opcode.three_nibble = pkt_hdr_type.three_nibble;
            				}
							

           					/* Now decode the packet payload */
            			    /* Decode the packet payload */
            				return_value = decode_packet_payload
                            							    ( list_of_bytes,
                                  							  length_of_bytes,
                                  							  current_byte_pos,
                                  							  current_nibble_pos,
                                  							  pkt_hdr_type,
                                  							  pkt_header_nibbles,
                                  							  device->version,
                                 							 &new_byte_pos,
                                 							 &new_nibble_pos,
															 &pkt_payload_decoded,
                                 							 &number_of_payload_nibbles,
                                  							  payload_data,
                                 							 &ts_raw_data.ts_nibble_length,
                                  							  ts_raw_data.ts_nibbles
                                 							);
            				

															
            				/* Update the current time */
            				if ((return_value == stp2_dec_success) && (pkt_payload_decoded))
            				{			
                				current_byte_pos    = new_byte_pos;
                				current_nibble_pos  = new_nibble_pos;

                				/* Update packet payload contents */
                				packets_decoded[pkt_decoded].payloadLength = number_of_payload_nibbles;
								
								packets_decoded[pkt_decoded].payload = 0;
								
								nibble_index = 0;
								if (number_of_payload_nibbles > 0)
								{
									nibble_index = (number_of_payload_nibbles - 1);
								}
								
								/* payload_data[0] = MSB nibble and payload_data[1] = LSB nibble */								
                				for (index=0; index < number_of_payload_nibbles; index++)
                				{  
                					nibble_data=0;
                					nibble_data = (payload_data[index]);
                					nibble_data = nibble_data << (nibble_index*4);
                					
                    				packets_decoded[pkt_decoded].payload |= nibble_data;

									nibble_index--;
                				}
               				

                				switch (pkt_header_nibbles)
                				{
                    				case 1: /* One nibble opcode */
                    				{
                        				if ( pkt_hdr_type.sng_nibble  == STP2_PKT_TYPE_M8 )
                        				{
										
                            				device->current_master_id  &= ~0xFF;
                            				device->current_master_id  |= ((payload_data[0] << 4) |
                                                          				   (payload_data[1]));
                            				device->current_channel_id = 0;

                            				pkt_type_to_be_stored = False;
                        				}
                        				else if ( pkt_hdr_type.sng_nibble  == STP2_PKT_TYPE_MERR )
                        				{
              								device->current_channel_id = 0;                            				
                            			

                        				}
                        				else if ( pkt_hdr_type.sng_nibble  == STP2_PKT_TYPE_C8 )
                        				{
                        					device->current_channel_id  &= ~0xFF;
                            				device->current_channel_id  |= ((payload_data[0] << 4) |
                                                            			    (payload_data[1]));

                            				pkt_type_to_be_stored = False;
                        				}

                        				break;
                    				}
                    				case 2: /* Two nibble opcode */
                    				{
                        				if ( pkt_hdr_type.two_nibble  == STP2_SEC_NIB_PKT_TYPE_GERR ) 
                        				{
                            				device->current_master_id  = 0;
                            				device->current_channel_id = 0;
                            				

                        				}
                        				else if ( pkt_hdr_type.two_nibble  == STP2_SEC_NIB_PKT_TYPE_M16 )
                        				{
                            				device->current_master_id  = 0;
                            				device->current_master_id  |= ((payload_data[0] << 12) |
                                                                           (payload_data[1] << 8 ) |
                                                                           (payload_data[2] << 4 ) |
                                                                           (payload_data[3]));
                            				device->current_channel_id = 0;

                            				pkt_type_to_be_stored = False;
                        				}                
                        				else if ( pkt_hdr_type.two_nibble  == STP2_SEC_NIB_PKT_TYPE_C16 )
                        				{
                            				device->current_channel_id  = 0;
                            				device->current_channel_id  |= ((payload_data[0]  << 12) |
                                                            				(payload_data[1]  << 8 ) |
                                                           					 (payload_data[2]  << 4 ) |
                                                            				 (payload_data[3]));
                            				pkt_type_to_be_stored = False;
                        				}
                        				break;
                    				}
                    				case 3: /* Three nibble opcode */
                    				{
                        				if ( pkt_hdr_type.three_nibble  == STP2_THR_NIB_PKT_TYPE_VERSION ) 
                        				{
                           					device->current_master_id  = 0;
                            				device->current_channel_id = 0;

                            				device->version = (enum stp2_version)(payload_data[0]);

                            				pkt_type_to_be_stored = False;
                        				}
                        				if ( pkt_hdr_type.three_nibble  == STP2_THR_NIB_PKT_TYPE_TSG ) 
                        				{
                            				device->current_TS_granularity = (enum stp2_version)(payload_data[0]);

                            				pkt_type_to_be_stored = False;
                        				}
										else if (( pkt_hdr_type.three_nibble  == STP2_THR_NIB_PKT_TYPE_FREQ) ||
			         							 ( pkt_hdr_type.three_nibble  == STP2_THR_NIB_PKT_TYPE_FREQ_TS))
										{
                            				device->timestamp_frequency = ( (payload_data[0]  << 28)   |
                                                           					(payload_data[1]  << 24 )  |
                                                            				(payload_data[2]  << 20 )  |
                                                            				(payload_data[3]  << 16 )  |
                                                            				(payload_data[4]  << 12 )  |
                                                            				(payload_data[5]  << 8 )   |
                                                            				(payload_data[6]  << 4 )   |
                                                            				(payload_data[7]) );

                            				pkt_type_to_be_stored = False;
										}
							            
                        				break;
                    				}
                    				default:
                    				{
                        				return_value = stp2_dec_err_internal_decoder_error;
										decode_error = True;
                    				}
									
                				} /* Switch statement */
								
								if (return_value == stp2_dec_success)
								{
									// Do not decode absolute timestamp if there is no TS packet being detected.
									// The TS granularity will come first, then the TS packet. We should wait
									// untile TS granularity packet comes out and use it for caculate absolute
									// timestamp.
									if( ts_raw_data.ts_nibble_length )
									{
										/* Update timestamp information of current stp2.0 packet */
               							return_value = update_absolute_timestamp
                                   									(               device->current_time,
																					device->current_TS_granularity,
                               														(const struct timestamp_nibbles *) &ts_raw_data,
                                      												device->version,
                                      							 					&new_time 
                                    								);
										// Reset current granularity.
										device->current_TS_granularity = 0;
									}
               						if (return_value == stp2_dec_success)
               						{
               							device->current_time = new_time;

               							/* Update packet timestamp contents */
               							packets_decoded[pkt_decoded].timestamp = device->current_time;
               						
									}
									else
									{
										decode_error = True;
									}
								}
								else
								{
									decode_error = True;
								}
							} /* if ((return_value == stp2_dec_success) && (pkt_payload_decoded)) */
							else if ((return_value == stp2_dec_success) && (!pkt_payload_decoded))
							{
								decode_error      = True;
								all_bytes_decoded = True;
							}
							else
							{
								decode_error      = True;
							}
						} /* If -- version packets needs not to be detected */
						  /* Needs to be fixed in case a partial version packet is seen */
						else
						{
							decode_error = True;
							return_value = stp2_dec_err_lost_sync;
						}
					}
					else if ((return_value == stp2_dec_success) && (pkt_hdr_info == PKT_HDR_DEC_ASYNC_START))
					{
            			/* Unwind the pointers and in the next while iteration look for sync sequence */
            			current_byte_pos            = start_byte_pointer;
            			current_nibble_pos          = start_nibble_pointer;
            			device->sync_sequence_found = False; 
	    				pkt_type_to_be_stored       = False;
						
						/* This is not a decode error and the next while loop iteration attempts to find the next sync seq */
					}
					else if ((return_value == stp2_dec_success) && (pkt_hdr_info == PKT_HDR_DEC_INSUFFICIENT_BYTES))
					{
						decode_error      = True;
						all_bytes_decoded = True;
					} 
					else
					{
						decode_error      = True;
					} /* if ((return_value == stp2_dec_success) && ( packet_hdr_info == )) */
			} /* if (current_byte_pos != list_of_bytes)*/
			else
			{
				all_bytes_decoded     = True;
				pkt_type_to_be_stored = False;
				partial_message_end   = False;
			}
		} /* if (device->sync_sequence_found == True) */
		else
		{
			decode_error = True;
		}
										
		if ((pkt_type_to_be_stored) && (!decode_error))
        {   /* If pkt does not need to be stored, then do not advance pkt decoded counter */	

            pkt_decoded++;
        }
			
	}	/* While statement */				
					
		
	if ((decode_error) || (return_value != stp2_dec_success))
	{
    	*next_byte_pos             = start_byte_pointer;
        *next_nibble_pos           = start_nibble_pointer;
        *number_of_packets_decoded = pkt_decoded;
    }
	else
	{
    	*next_byte_pos             = current_byte_pos;
    	*next_nibble_pos           = current_nibble_pos;
    	*number_of_packets_decoded = pkt_decoded;
    }
	
	*end_of_input_buffer = all_bytes_decoded;
	*partial_message     = partial_message_end;

    return return_value;
}

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     search_async_sequence   
 *
 * @BRIEF        This API attempts to find either the first or subsequent ASYNC
 *               pattern in data stream. 
 *
 * @DESCRIPTION  This API attempts to find either the first or subsequent ASYNC
 *               pattern in data stream. In case of a subsequent ASYNC sequence,
 *               This must be found in the first 22 nibbles else it is reported as 
 *               not found. In case of the first ASYNC sequence, the nibbles are scanned
 *               until 22 consecutive nibbles have the ASYNC pattern. If one cannot
 *               be identified then the API returns to the caller.
 *
 * @RETURNS      Description of return value
 *                       stp2_dec_err_internal_decoder_error
 *                       stp2_dec_err_lost_sync
 *                       stp2_dec_success
 * 
 *//*------------------------------------------------------------------------ */
static enum return_code_stp2_dec search_async_sequence 
                            ( const u8              *list_of_bytes,    /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                    number_of_bytes,  /* Total number of bytes in the caller's buffer */
                              BOOL                   first_async_search, /* FALSE = first async pattern search and TRUE = Subsequent async pattern search */
                              u32                    byte_pos,         /* The position of the next byte to be decoded */
                              enum stp2_nibble_pos   nibble_pos,       /* FALSE = next nibble to be decoded is bits 3:0; True = bits 7:4 */
                              u32                   *new_byte_pos,     /* This API returns the position of next byte to be decoded after async detection and extraction */
                              enum stp2_nibble_pos  *new_nibble_pos,   /* This API returns the position of the next nibble to be decoded after async detection and extraction */
							  BOOL                  *partial_async,    /* This API returns if a partial async was found or not */
							  BOOL                  *async_found)      /* This parameter returns whether an async sequence is found or not */
{
    u32             	  current_byte_pos              = byte_pos;
    enum stp2_nibble_pos  current_nibble_pos            = nibble_pos;
    BOOL                  async_sequence_found          = False;
    BOOL                  async_sequence_end            = False;
    BOOL              	  end_of_buffer_found           = False;
    u8                	  async_pattern_length_detected = 0;
    u8                	  byte_to_decode                = 0;
    u8                	  nibble_to_decode              = 0;

    
    /* Null parameter check */
    if ((list_of_bytes   == NULL)   || 
        (new_byte_pos    == NULL)   || 
        (new_nibble_pos  == NULL)   || 
		(partial_async   == NULL)   || 
		(async_found     == NULL)   ||
		(byte_pos        >= number_of_bytes)) 
    {
        return stp2_dec_err_internal_decoder_error;
    }


    /* Defensive return values */
    *async_found     = False;
    *new_nibble_pos  = nibble_pos;
    *new_byte_pos    = byte_pos;
	*async_found     = False;
	*partial_async   = False;

    /* Local Variable initialization */
    async_pattern_length_detected = 0;
    end_of_buffer_found           = False;
    async_sequence_found          = False;

    /* Async pattern is 21 0xF's followed by a 0x0 */
    while ((async_sequence_end == False) && (end_of_buffer_found == False))
    {	   
       if( current_byte_pos == number_of_bytes )
       {
            end_of_buffer_found = True;
			break;
       }
	   
       byte_to_decode   = list_of_bytes[current_byte_pos];
       nibble_to_decode = NIBBLE (byte_to_decode, current_nibble_pos);

       current_byte_pos++;
       if (async_pattern[async_pattern_length_detected] == nibble_to_decode)
       {
            // If sequence of 21 0xF is found and the current one is a 0
		    if( async_sequence_found )
				async_sequence_end = True;
            async_pattern_length_detected++;
       }
	   // If it is the first time seeking async sequence, the first a couple of leading 0xF's may be
	   // the ones from previous packet. Therefore, the number of 0xF for the async could be more than
	   // STP2_ASYNC_PATTERN_F_LENGTH. For the first async sequence we don't restrict the sequence value
	   // exact as what defind in async_pattern, but validate it as far as it is consecutive 0xF's 
	   else if( first_async_search && async_sequence_found && ( async_pattern[0] == nibble_to_decode ) )
	   {
            async_pattern_length_detected++;
	   }
       else
       {
			async_pattern_length_detected = 0;
       }

       // If sequence of 21 0xF is found, set the flag.
       if (async_pattern_length_detected == STP2_ASYNC_PATTERN_F_LENGTH)
       {
            async_sequence_found = True;
       }


       if ((async_pattern_length_detected == 0) && (first_async_search == False))
       {   /* Must stop the search if this is not the first attempt to look for an async pattern */
           break;
       }
    }

    if (async_sequence_found == True)
    {
        *new_nibble_pos  = current_nibble_pos;
        *new_byte_pos    = current_byte_pos;
		*partial_async   = False;
		*async_found     = True;
    }
	else if (async_pattern_length_detected > 0)
	{
	    *partial_async   = True;
		*async_found     = False;
	}
	else
	{
		*partial_async   = False;
		*async_found     = False;
	}
	
    return stp2_dec_success;
}

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     decode_packet_header    
 *
 * @BRIEF        This API attempts to decode an STP2.0 packet header
 *               It may find a packet header or the start of an async sequence or
 *               it may not find any packet header at all.
 *
 * @DESCRIPTION  This API looks for 1, 2 or 3 nibble STP 2.0 packets and advances
 *               the (byte,nibble) pointer pair to the next nibble beyond the packet
 *               decoded if one is indeed decoded. It may identify instead the first
 *               three nibbles of an async sequence. If this is the case, the 
 *               (byte,nibble) pointer pair that API returns to caller are unwound
 *               so that an attempt can be made to decode the complete async sequence.
 *
 * @RETURNS      Description of return value
 *                               stp2_dec_success
 *                               stp2_dec_err_lost_sync
 *                               stp2_dec_err_internal_decoder_error
 *                               stp2_dec_err_array_index_out_of_range
 *                               stp2_dec_err_null_parameter
 *                         
 *                         
 
 *
 * @NOTES        STP2.0 NULL packets are filtered out by this API
 * 
 *//*------------------------------------------------------------------------ */
static enum return_code_stp2_dec decode_packet_header 
                            ( const u8                    *list_of_bytes,           /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                          number_of_bytes,         /* Total number of bytes in the caller's buffer */
                              u32                          byte_pos,                /* The position of the next byte to be decoded */
                              enum stp2_nibble_pos         nibble_pos,              /* The position of the current nibble to be decoded */
                              u32                         *new_byte_pos,            /* This API returns the position of next byte to be decoded after packet header extraction  */
                              enum stp2_nibble_pos        *new_nibble_pos,          /* This API returns the position of the next nibble to be decoded after packet header extraction */
							  enum pkt_hdr_dec_info       *pkt_hdr_decoded_info,    /* Returns whether pkt has been decoded, insufficient bytes to decode or start of async sequence */
                              union stp2_pkt_header_type  *pkt_header_type,         /* Returns the type of packet found */
                              u8                          *pkt_num_nibbles )        /* Returns the number of nibbles found in packet */
{
    u32                                 current_byte_pos       = byte_pos;
    enum stp2_nibble_pos                current_nibble_pos     = nibble_pos;
    u8                                  byte_to_decode         = 0;
    u8                                  nibble_to_decode       = 0;
    enum stp2_sng_nibble_packet_type    sng_nibble_pkt         = STP2_PKT_TYPE_NONE;
    enum stp2_two_nibble_packet_type    two_nibble_pkt         = STP2_SEC_NIB_PKT_TYPE_EXT_OPC;
    enum stp2_three_nibble_packet_type  three_nibble_pkt       = STP2_THR_NIB_PKT_TYPE_RESERVED3;
    u8                                  num_pkt_nibbles_found  = 0;
	enum pkt_hdr_dec_info               pkt_hdr_info           = PKT_HDR_DEC_RESERVED;
    union stp2_pkt_header_type          pkt_header;
                                        pkt_header.sng_nibble  = STP2_PKT_TYPE_NONE;
	enum  return_code_stp2_dec          return_code;
   
    if ( (list_of_bytes         == NULL) || 
         (new_byte_pos          == NULL) ||
         (new_nibble_pos        == NULL) ||
         (pkt_num_nibbles       == NULL) ||
		 (pkt_hdr_decoded_info  == NULL) ||
		 (pkt_header_type       == NULL) ||
		 (current_byte_pos      >= number_of_bytes) )
    {
        return stp2_dec_err_internal_decoder_error;
    }


    /* Set the return values */
    *new_byte_pos         = byte_pos;
    *new_nibble_pos       = nibble_pos;
	*pkt_hdr_decoded_info = PKT_HDR_DEC_RESERVED;
 
    *pkt_num_nibbles      = 0;

    num_pkt_nibbles_found = 0;
	return_code           = stp2_dec_success;
	
	/* Null packets are disgarded in the search for a pkt header */
	/* There are a couple of scenarios */
	/* 1) Stop packet detection in middle of a packet header 
	 * 2) Detect only NULL packets 
	 * 3) Detect the start of the ASYNC sequence
	 * 4) Don't detect any packet header which decodes to a proper STP2 packet
	 */


    /* Search the byte stream until packet header is found or start of sync sequence or error or input byte buffer is exhausted */
    while ((pkt_hdr_info == PKT_HDR_DEC_RESERVED) && (return_code == stp2_dec_success))
    {
        if (current_byte_pos != number_of_bytes)
        {
        	byte_to_decode   = list_of_bytes[current_byte_pos];
        	nibble_to_decode = NIBBLE(byte_to_decode, current_nibble_pos);

        	sng_nibble_pkt   = (enum stp2_sng_nibble_packet_type)nibble_to_decode;
        	two_nibble_pkt   = (enum stp2_two_nibble_packet_type)nibble_to_decode;
        	three_nibble_pkt = (enum stp2_three_nibble_packet_type)nibble_to_decode;
			
			current_byte_pos++;

        
        	switch (num_pkt_nibbles_found)
        	{
            	case 0: /* First nibble of a pkt */
            	{
                	/* Let's decode the nibble and see what we have */
                	if ((sng_nibble_pkt != STP2_PKT_TYPE_NONE) &&
                        (sng_nibble_pkt != STP2_PKT_TYPE_EXT_OPC))
                	{
                    	/* We have a valid header and we can stop */
                    	pkt_hdr_info = PKT_HDR_DEC_SUCCESS;
                    	num_pkt_nibbles_found = 1;
                    	pkt_header.sng_nibble = sng_nibble_pkt;
						// If it is a master packet, exit for reading payload master ID.
						if( STP2_PKT_TYPE_M8 == sng_nibble_pkt )
							break;
                	}
                	else if (sng_nibble_pkt == STP2_PKT_TYPE_EXT_OPC)
                	{   /* Extension opcode, we must continue */
                    	num_pkt_nibbles_found = 1;
                	}
                	else
                	{   /* NULL packet and we haven't found any packet nibbles and must continue from scratch 
					     * unless we must stop because NULL packet is at end of the input byte buffer 
						 */
                    	num_pkt_nibbles_found = 0;
						
						if (current_byte_pos == number_of_bytes)
						{   /* There are no more bytes to decode so exit the loop */
					    	pkt_hdr_info          = PKT_HDR_DEC_SUCCESS;
							num_pkt_nibbles_found = 1;
							pkt_header.sng_nibble = sng_nibble_pkt;	
						}
                	}
                	break;
            	}
            	case 1: /* Second nibble of a pkt */
            	{
                	/* Let's decode the nibble and see what we have */
                	if ((two_nibble_pkt != STP2_SEC_NIB_PKT_TYPE_EXT_OPC) &&
                    	(two_nibble_pkt != STP2_SEC_NIB_PKT_TYPE_RESERVED0) &&
                    	(two_nibble_pkt != STP2_SEC_NIB_PKT_TYPE_ASYNC_SRT))
                	{
                    	/* We have a valid header and we can stop */
                    	pkt_hdr_info          = PKT_HDR_DEC_SUCCESS;
                    	num_pkt_nibbles_found = 2;
                    	pkt_header.two_nibble = two_nibble_pkt;

                	}
                	else if (two_nibble_pkt == STP2_SEC_NIB_PKT_TYPE_ASYNC_SRT)
                	{   /* Possible async packet detected */
                    	pkt_hdr_info          = PKT_HDR_DEC_ASYNC_START;
                    	num_pkt_nibbles_found = 0;
                	}		
                	else if (two_nibble_pkt == STP2_SEC_NIB_PKT_TYPE_EXT_OPC)
                	{   /* Extension opcode, we must continue */
                    	num_pkt_nibbles_found = 2;
                	}
                	else
                	{   /* An illegal opcode has been detected */
                    	return_code =  stp2_dec_err_lost_sync;
                	}
                	break;
            	}
            	case 2: /* Third nibble of a pkt */
            	{
                	/* Let's decode the nibble and see what we have */
                	if ((three_nibble_pkt != STP2_THR_NIB_PKT_TYPE_RESERVED3) &&
                    	(three_nibble_pkt != STP2_THR_NIB_PKT_TYPE_RESERVED0) &&
                    	(three_nibble_pkt != STP2_THR_NIB_PKT_TYPE_RESERVED1) &&
                    	(three_nibble_pkt != STP2_THR_NIB_PKT_TYPE_RESERVED2))
                	{
                    	/* We have a valid header and we can stop */
                    	pkt_hdr_info            = PKT_HDR_DEC_SUCCESS;
                    	num_pkt_nibbles_found   = 3;
                    	pkt_header.three_nibble = three_nibble_pkt;

                	}
                	else
                	{   /* An illegal opcode has been detected */
                    	return_code = stp2_dec_err_lost_sync;
                	}
                	break;
            	}
            	default:
            	{
                	return_code = stp2_dec_err_internal_decoder_error;
            	}
        	}
		} /* If then else */
		else
		{
			pkt_hdr_info = PKT_HDR_DEC_INSUFFICIENT_BYTES;
    	}

    } /* While */
	

    if (return_code == stp2_dec_success) 
	{
		/* We exited the loop without error if either a packet header was found 
		 * or an async start sequence was detected or there are insufficient bytes
		 * to complete decode
		 */
		
		/* We exited the loop because either a packet header was found or async sequence start */
    	if ((pkt_hdr_info == PKT_HDR_DEC_ASYNC_START) || 
		    (pkt_hdr_info == PKT_HDR_DEC_INSUFFICIENT_BYTES))
    	{
        	/* Set the return values */
        	*new_byte_pos         = byte_pos;
       	 	*new_nibble_pos       = nibble_pos;
			*pkt_num_nibbles      = 0; /* Since the byte pointers are being unwound */
    	}
    	else if (pkt_hdr_info == PKT_HDR_DEC_SUCCESS)
    	{
        	/* Set the return values */
        	*new_byte_pos         = current_byte_pos;
        	*new_nibble_pos       = current_nibble_pos;
        	*pkt_num_nibbles      = num_pkt_nibbles_found;

        	if (num_pkt_nibbles_found == 1)
        	{
            	pkt_header_type->sng_nibble = pkt_header.sng_nibble;
       	 	}
        	else if (num_pkt_nibbles_found == 2)
        	{
            	pkt_header_type->two_nibble  = pkt_header.two_nibble;
        	}
        	else if (num_pkt_nibbles_found == 3)
        	{
            	pkt_header_type->three_nibble  = pkt_header.three_nibble;
        	}
        	else
        	{
            	return_code = stp2_dec_err_internal_decoder_error;
        	}		
		}
		else
		{
			return_code = stp2_dec_err_internal_decoder_error;
		}
    }
    else
    {
        /* Set the return values */
        *new_byte_pos         = byte_pos;
       	*new_nibble_pos       = nibble_pos;
		*pkt_num_nibbles      = 0; /* Since the byte pointers are being unwound */        
    }
	
	*pkt_hdr_decoded_info = pkt_hdr_info;

    return return_code;
}


/*-------------------------------------------------------------------------*//**
 * @FUNCTION     decode_packet_payload 
 *
 * @BRIEF        This API attempts to decode an STP2.0 packet payload

 *
 * @DESCRIPTION  
 *
 * @RETURNS      Description of return value
 *                               stp2_dec_success
 *                               stp2_dec_err_lost_sync
 *                               stp2_dec_err_internal_decoder_error
 *                               stp2_dec_err_array_index_out_of_range
 *                               stp2_dec_err_null_parameter
 *                               
 * 
 *//*------------------------------------------------------------------------ */
static enum return_code_stp2_dec decode_packet_payload 
                            ( const u8                    *list_of_bytes,                /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                          number_of_bytes,              /* Total number of bytes in the caller's buffer */
                              u32                          byte_pos,                     /* The position of the next byte to be decoded */
                              enum  stp2_nibble_pos        nibble_pos,                   /* The position of the current nibble to be decoded */
                              union stp2_pkt_header_type   pkt_header_type,              /* The type of packet header detected  */
                              u8                           pkt_header_num_nibbles,       /* The number of nibbles in the packet header */
                              enum stp2_version            version,                      /* The current STP2.0 version number */
                              u32                         *new_byte_pos,                 /* This API returns the position of next byte to be decoded after packet header extraction  */
                              enum stp2_nibble_pos        *new_nibble_pos,               /* This API returns the position of the next nibble to be decoded after packet header extraction */
							  BOOL                        *pkt_payload_decoded,          /* Returns whether payload has been decoded or not */
                              u8                          *number_of_payload_nibbles,    /* This API returns the number of payload nibbles detected */
                              u8                          *payload_data,                 /* This API puts the payload nibbles into caller's buffer */
                              u8                          *number_of_timestamp_nibbles,  /* This API returns the number of timestamp nibbles found */
                              u8                          *timestamp_nibble_values )     /* This API puts the timestamp nibbles into caller's buffer */

{
    u32                     	 current_byte_pos                  = byte_pos;
    enum stp2_nibble_pos    	 current_nibble_pos                = nibble_pos;
    u8                      	 byte_to_decode                    = 0;
    u8                      	 nibble_to_decode                  = 0;
    u8                      	 number_of_nibbles_payload_extract = 0;
    BOOL                    	 number_of_nibbles_payload_known   = False;
    BOOL                    	 timestamp_to_extract              = False;
	BOOL                    	 timestamp_decoded                 = False;
    u8                      	 index                             = 0;
    u32                     	 after_timestamp_byte_pos          = byte_pos;
    enum stp2_nibble_pos    	 after_timestamp_nibble_pos        = nibble_pos;
    u8                      	 num_ts_nibbles                    = 0;
    enum return_code_stp2_dec    return_value                      = stp2_dec_success;

	
    /* Check error conditions */
    if ( (list_of_bytes                == NULL) || 
         (new_byte_pos                 == NULL) ||
         (new_nibble_pos               == NULL) ||
         (number_of_payload_nibbles    == NULL) ||
         (payload_data                 == NULL) ||
         (timestamp_nibble_values      == NULL) ||
         (number_of_timestamp_nibbles  == NULL) ||
         (timestamp_nibble_values      == NULL) ||
		 (pkt_payload_decoded          == NULL))
    {
        return stp2_dec_err_internal_decoder_error;
    }


    /* Set the return values */
    *new_byte_pos                = byte_pos;
    *new_nibble_pos              = nibble_pos;
    *number_of_payload_nibbles   = 0;
    *number_of_timestamp_nibbles = 0;
	*pkt_payload_decoded         = True; /* Indicate success unless otherwise indicated */


    number_of_nibbles_payload_extract = 0;
    number_of_nibbles_payload_known   = False;  /* This will be set to true only if payload length is known */
    timestamp_to_extract              = False;



    /* Extract the payload */
    switch (pkt_header_num_nibbles)
    {
        case 0: /* No nibbles */
        {
            return_value = stp2_dec_err_internal_decoder_error;
            break;
        }
        case 1: /* One nibble in packet */
        {
            /* Let's decode the nibble and see what we have */
            if ( stp_sng_packet_attributes[pkt_header_type.sng_nibble].payload_length_known )
            {
                number_of_nibbles_payload_extract = stp_sng_packet_attributes[pkt_header_type.sng_nibble].payload_length;
                number_of_nibbles_payload_known   = True;
				return_value                      = stp2_dec_success;
            }
            else
            {
                return_value = stp2_dec_err_lost_sync;
            } 

            timestamp_to_extract  = stp_sng_packet_attributes[pkt_header_type.sng_nibble].timestamp_present;
            break;
        }
        case 2: /* Two nibbles of a pkt */
        {
            /* Let's decode the nibble and see what we have */
            if ( stp_two_nibble_packet_attributes[pkt_header_type.two_nibble].payload_length_known )
            {
                number_of_nibbles_payload_extract = stp_two_nibble_packet_attributes[pkt_header_type.two_nibble].payload_length;
                number_of_nibbles_payload_known   = True;
				return_value                      = stp2_dec_success;
            }
            else
            {
                return_value = stp2_dec_err_lost_sync;
            } 
            timestamp_to_extract = stp_two_nibble_packet_attributes[pkt_header_type.two_nibble].timestamp_present;
            break;
        }
        case 3: /* Three nibbles of a pkt */
        {
            /* Let's decode the nibble and see what we have */
            if ( stp_three_nibble_packet_attributes[pkt_header_type.three_nibble].payload_length_known )
            {
                number_of_nibbles_payload_extract = stp_three_nibble_packet_attributes[pkt_header_type.three_nibble].payload_length;
                number_of_nibbles_payload_known   = True;
				return_value                      = stp2_dec_success;
            }
            timestamp_to_extract = stp_three_nibble_packet_attributes[pkt_header_type.three_nibble].timestamp_present;
            break;
        }
        default: /* Should never happen */
        {
            return_value = stp2_dec_err_lost_sync;
        }

    }

	if ((return_value == stp2_dec_success) && (*pkt_payload_decoded))
	{
    	if  (!number_of_nibbles_payload_known)
    	{
			if (current_byte_pos != number_of_bytes)
        	{
       	    	byte_to_decode      = list_of_bytes[current_byte_pos];
            	nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

		        current_byte_pos++;
				       
        		/* The exception cases are for USER packet and TIME packet in terms of the payload data */
       			if (((pkt_header_num_nibbles == 3) && (pkt_header_type.three_nibble == STP2_THR_NIB_PKT_TYPE_USER)) ||
           			((pkt_header_num_nibbles == 3) && (pkt_header_type.three_nibble == STP2_THR_NIB_PKT_TYPE_USER_TS)))
       			{
            		/* The nibble_to_decode + 1 is the total number of nibbles of payload to extract */
            		number_of_nibbles_payload_known   = True;
            		number_of_nibbles_payload_extract = nibble_to_decode + 1;
       			}
       			else if (((pkt_header_num_nibbles == 3) && (pkt_header_type.three_nibble == STP2_THR_NIB_PKT_TYPE_TIME)) ||
                		((pkt_header_num_nibbles == 3) && (pkt_header_type.three_nibble == STP2_THR_NIB_PKT_TYPE_TIME_TS)))
       			{
            		/* The first nibble is the clock index and this is currently thrown away -- need to fix it */
					if (current_byte_pos != number_of_bytes)
					{
            			/* The second nibble is the version info for how to extract the payload */
           	 			byte_to_decode      = list_of_bytes[current_byte_pos];
            			nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

                        current_byte_pos++;
						
        				/* Since it is a TS payload using the timestamp decode function to grab the payload data */
        				after_timestamp_byte_pos    = current_byte_pos;
        				after_timestamp_nibble_pos  = current_nibble_pos;
        				num_ts_nibbles              = 0;

            			return_value = decode_packet_timestamp (list_of_bytes,
                                              			      	number_of_bytes,
                                                    			current_byte_pos,
                                                    			current_nibble_pos,
                                 			 (enum stp2_version)nibble_to_decode, /* Version number */
                                                    			&after_timestamp_byte_pos,
                                                    			&after_timestamp_nibble_pos,
																&timestamp_decoded,
                                                    			&num_ts_nibbles, 
                                                    			payload_data);
																																													
						/* Indicate the pkt payload cannot be decoded */							
						if ((return_value == stp2_dec_success) && (!timestamp_decoded))
						{		
							*pkt_payload_decoded = False;
						}
						else if (return_value == stp2_dec_success)
						{
							/* Return parameter of number of payload nibbles extracted */
            				*number_of_payload_nibbles = num_ts_nibbles;

            				/* Update the current byte and nibble pointers */
            				current_byte_pos    = after_timestamp_byte_pos;
            				current_nibble_pos  = after_timestamp_nibble_pos;
							
							/* Since payload has already been extracted, it doesn't need to be 
							 * later
							 */
							number_of_nibbles_payload_known =  False;
														
						}					
					}
					else
					{
						*pkt_payload_decoded = False;
					}					
				}
				else
				{
					return_value = stp2_dec_err_internal_decoder_error;
				}					
			}
			else
			{
				*pkt_payload_decoded = False;
			}
		}
	}
	
	if ((return_value == stp2_dec_success) && (*pkt_payload_decoded) && (number_of_nibbles_payload_known))
	{
	                                    
    	/* Fall through in all cases to extract the payload when length is now known */
    	/* Only known exception whether length is not known is for TIME, TIME_TS packets where the payload has 
     	 * already been previously extracted.
     	 */
        *number_of_payload_nibbles = number_of_nibbles_payload_extract;

        for (index=0; index < number_of_nibbles_payload_extract; index++)
        {			
            if (current_byte_pos != number_of_bytes)
            {
            	byte_to_decode      = list_of_bytes[current_byte_pos];
            	nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

            	payload_data[index] = nibble_to_decode; 
            
                current_byte_pos++;
			}
			else
			{
				*pkt_payload_decoded = False;
				break;
			}
        }
    }
	
	/* Extract the timestamp */
	if ((return_value == stp2_dec_success) && (*pkt_payload_decoded) && (timestamp_to_extract))
	{	
        return_value = decode_packet_timestamp (list_of_bytes,
                                                number_of_bytes,
                                                current_byte_pos,
                                                current_nibble_pos,
                                                version, /* Version number */
                                                &after_timestamp_byte_pos,
                                                &after_timestamp_nibble_pos,
												&timestamp_decoded,
                                                number_of_timestamp_nibbles, 
                                                timestamp_nibble_values);

        if ((return_value == stp2_dec_success) && (timestamp_decoded))
        {
        	/* Update byte and nibble pointers for the caller after timestamp extraction */
        	*new_byte_pos   = after_timestamp_byte_pos;
        	*new_nibble_pos = after_timestamp_nibble_pos;			
	    }
		else if ((return_value == stp2_dec_success) && (!timestamp_decoded))
		{
		    /* Input buffer has been consumed */
			*pkt_payload_decoded = False;
		}
		
    }
    else if ((return_value == stp2_dec_success) && (*pkt_payload_decoded) && (!timestamp_to_extract))
    {
      
        *number_of_timestamp_nibbles = 0x0;
        *new_byte_pos                = current_byte_pos;
        *new_nibble_pos              = current_nibble_pos;         
	}
	
	
    return return_value;
}

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     decode_packet_timestamp
 *
 * @BRIEF        This API attempts to decode an STP2.0 packet timestamp

 *
 * @DESCRIPTION  This API takes a timestamp version information and parses the
 *               byte stream to return a timestamp in any available STP2.0 format.
 *
 * @RETURNS      Description of return value
 *                         stp2_dec_success
 *                         stp2_dec_err_internal_decoder_error
 *                         stp2_dec_err_lost_sync
 *                                       
 * 
 *//*------------------------------------------------------------------------ */
static enum return_code_stp2_dec decode_packet_timestamp 
                             (const u8                    *list_of_bytes,                /* Buffer pointing to list of bytes from caller to be decoded */
                              u32                          number_of_bytes,              /* Total number of bytes in the caller's buffer */
                              u32                          byte_pos,                     /* The position of the next byte to be decoded */
                              enum  stp2_nibble_pos        nibble_pos,                   /* The position of the current nibble to be decoded */
                              enum stp2_version            version,                      /* The current STP2.0 version number format of the timestamp to be extracted */
                              u32                         *new_byte_pos,                 /* This API returns the position of next byte to be decoded after timestamp extraction  */
                              enum stp2_nibble_pos        *new_nibble_pos,               /* This API returns the position of the next nibble to be decoded after timestamp extraction */
							  BOOL                        *timestamp_decoded,            /* Whether the timestamp has been decoded or not */
                              u8                          *number_of_timestamp_nibbles,  /* This API returns the number of timestamp nibbles found */
                              u8                          *timestamp_nibble_values )     /* This API puts the timestamp nibbles into caller's buffer */         
                              

{
    
    u32                    current_byte_pos   = byte_pos;
    enum stp2_nibble_pos   current_nibble_pos = nibble_pos;
    u8                	   byte_to_decode     = 0;
    u8                     nibble_to_decode   = 0;
    u8                     index              = 0;
    u8                     timestamp_len      = 0;

    
	/* Check for errors in input parameters */
    if ( (list_of_bytes                == NULL) || 
         (new_byte_pos                 == NULL) ||
         (new_nibble_pos               == NULL) ||
         (number_of_timestamp_nibbles  == NULL) ||
         (timestamp_nibble_values      == NULL) ||
		 (timestamp_decoded            == NULL) )
    {
        return stp2_dec_err_internal_decoder_error;
    }

	
	/* Default values for the return parameters */
    *new_byte_pos                = byte_pos;
    *new_nibble_pos     	     = nibble_pos;
	*timestamp_decoded  		 = False;
	*number_of_timestamp_nibbles = False;

 
    switch (version)
    {
        case  STP2_VERSION_V1_A:
        case  STP2_VERSION_V1_B:
        {   /* 2 nibble timestamp */
            if (current_byte_pos != number_of_bytes)
            {
            	byte_to_decode      = list_of_bytes[current_byte_pos];
            	nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

            	timestamp_nibble_values[0] = nibble_to_decode; 
            
                current_byte_pos++;
            
            	if (current_byte_pos != number_of_bytes)
            	{
            		byte_to_decode      = list_of_bytes[current_byte_pos];
            		nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

            		timestamp_nibble_values[1] = nibble_to_decode; 
            
					 current_byte_pos++;

            		*number_of_timestamp_nibbles = 2;
            		*new_byte_pos                = current_byte_pos;
            		*new_nibble_pos              = current_nibble_pos;
					*timestamp_decoded           = True;
				}
				else
				{
					*timestamp_decoded = False;
				}
        	}
			else
			{
				*timestamp_decoded = False;
			}
			break;
		}
        case  STP2_VERSION_V2_NATDELTA:
        case  STP2_VERSION_V2_NAT:
        case  STP2_VERSION_V2_GRAY:
        {
            if (current_byte_pos != number_of_bytes)
            {
            	byte_to_decode      = list_of_bytes[current_byte_pos];
            	nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

				current_byte_pos++;

            	timestamp_len = nibble_to_decode;
            	
            	
            	
               /* add time stamp */
                if (nibble_to_decode == 13)
            	{
                	timestamp_len = 14;
           		}
            	else if (nibble_to_decode == 14)
            	{
                	timestamp_len = 16;
            	}
            	else if (nibble_to_decode != 15)
            	{
                	timestamp_len = nibble_to_decode;
            	}
            	else
            	{
                	return stp2_dec_err_lost_sync;
            	}

				*timestamp_decoded = True; /* It will be set to false if not enough bytes to decode */
            	for (index=0; index < timestamp_len; index++)
            	{
                	if (current_byte_pos != number_of_bytes)
                	{
                		byte_to_decode      = list_of_bytes[current_byte_pos];
                		nibble_to_decode    = NIBBLE(byte_to_decode, current_nibble_pos);

                		timestamp_nibble_values[index] = nibble_to_decode; 

						current_byte_pos++;
            		}
					else
					{
					   *timestamp_decoded = False;
						break;
					}
				}
				
				if (*timestamp_decoded)
				{
            		*number_of_timestamp_nibbles = timestamp_len;
            		*new_byte_pos                = current_byte_pos;
            		*new_nibble_pos              = current_nibble_pos;
				}
			} /* if */
			else
			{
				*timestamp_decoded = False;
			}
			break;
        }
        default:
        {
            return stp2_dec_err_internal_decoder_error;
        }
    }

    return stp2_dec_success;
}

/*-------------------------------------------------------------------------*//**
 * @FUNCTION     update_absolute_timestamp
 *
 * @BRIEF        This API attempts to update the current time

 *
 * @DESCRIPTION  This API takes a timestamp version information and parses the
 *               byte stream to return a timestamp in any available STP2.0 format.
 *
 * @RETURNS      Description of return value
 *
 * 
 *//*-------------------------------------------------------------------------*/
static enum return_code_stp2_dec update_absolute_timestamp 
                             (const u64                   		  current_time,   /* This is the current absolute time (binary) */
							  const u64                      current_granularity, /* This is the current cranularity    */
                              const struct timestamp_nibbles     *raw_timestamp,  /* This is up to 16 nibbles of raw timestamp and len information */
                                    enum stp2_version             version,        /* This is the version information */
                                    u64                   	 	 *new_time)       /* This API returns the new time into caller's buffer */
{
    u64 new_value=0;
	u64 time_tp;
	u64 mask = 0;

	/* Check input parameter values */
    if ( (raw_timestamp               == NULL) ||
         (new_time                    == NULL) )
    {
        return stp2_dec_err_internal_decoder_error;
    }
    

    /* Set the new time equal to the old time */
	*new_time    = current_time;

   /* This function converts the timestamp nibble stream into an update on the 64-bit absolute timestamp tracked 
	* Only the STP2 VERSION (NAT) is currently supported 
	* This is the only format seen in any known hardware implementation on TI platforms other than STPv2NATDELTA 
	* Support for STPv2NATDELTA (MIPI_STM OMAP5430 ES2.0) is to be added 
	*/
    switch (version)
    {
	    case STP2_VERSION_V2_GRAY:
			new_value = bin_to_gray( current_time );
        case STP2_VERSION_V2_NAT:
        case STP2_VERSION_V2_NATDELTA:
        {
            switch (raw_timestamp->ts_nibble_length)
            {
                case 0:
                {
                    *new_time = current_time;
                    break;
                }
                case 1:
                {
				    mask      = 0xF;
					new_value &= ~mask;
                    new_value |= (raw_timestamp->ts_nibbles[0])&0xF;
                    break;
                }
                case 2:
                {
					mask      = 0xFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF) << 4) | 
                                 (raw_timestamp->ts_nibbles[1]&0xF));
                    break;
                }
                case 3:
                {
				    mask      = 0xFFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF) << 8) | 
                                 ((raw_timestamp->ts_nibbles[1]&0xF) << 4) |
                                  (raw_timestamp->ts_nibbles[2]&0xF));
                    break;
                }
                case 4:
                {
					mask      = 0xFFFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF)  << 12) | 
                                 ((raw_timestamp->ts_nibbles[1]&0xF)  << 8)  |
                                 ((raw_timestamp->ts_nibbles[2]&0xF)  << 4)  |
                                  (raw_timestamp->ts_nibbles[3]&0xF));
                    break;
                }
                case 5:
                {
					mask      = 0xFFFFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF)  << 16) | 
                                 ((raw_timestamp->ts_nibbles[1]&0xF)  << 12) |
                                 ((raw_timestamp->ts_nibbles[2]&0xF)  << 8)  |
                                 ((raw_timestamp->ts_nibbles[3]&0xF)  << 4)  |
                                 (raw_timestamp->ts_nibbles[4]&0xF));
                    break;
                }
                case 6:
                {
					mask      = 0xFFFFFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF)  << 20) | 
                                 ((raw_timestamp->ts_nibbles[1]&0xF)  << 16) |
                                 ((raw_timestamp->ts_nibbles[2]&0xF)  << 12) |
                                 ((raw_timestamp->ts_nibbles[3]&0xF)  << 8)  |
                                 ((raw_timestamp->ts_nibbles[4]&0xF)  << 4)  |
                                  (raw_timestamp->ts_nibbles[5]&0xF));
                    break;
                }
                case 7:
                {
					mask      = 0xFFFFFFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF)  << 24) | 
                                 ((raw_timestamp->ts_nibbles[1]&0xF)  << 20) |
                                 ((raw_timestamp->ts_nibbles[2]&0xF)  << 16) |
                                 ((raw_timestamp->ts_nibbles[3]&0xF)  << 12) |
                                 ((raw_timestamp->ts_nibbles[4]&0xF)  << 8)  |
                                 ((raw_timestamp->ts_nibbles[5]&0xF)  << 4)  |
                                  (raw_timestamp->ts_nibbles[6]&0xF));
                    break;
                }
                case 8:
                {
					mask      = 0xFFFFFFFF;
					new_value &= ~mask;
                    new_value |= (((raw_timestamp->ts_nibbles[0]&0xF)  << 28) | 
                                 ((raw_timestamp->ts_nibbles[1]&0xF)  << 24) |
                                 ((raw_timestamp->ts_nibbles[2]&0xF)  << 20) |
                                 ((raw_timestamp->ts_nibbles[3]&0xF)  << 16) |
                                 ((raw_timestamp->ts_nibbles[4]&0xF)  << 12) |
                                 ((raw_timestamp->ts_nibbles[5]&0xF)  << 8)  |
                                 ((raw_timestamp->ts_nibbles[6]&0xF)  << 4)  |
                                  (raw_timestamp->ts_nibbles[7]&0xF));
                    break;
                }
                case 9:
                {
					mask      = 0xFFFFFFFFFULL;
					new_value &= ~mask;
                    new_value |= ((((u64)(raw_timestamp->ts_nibbles[0]&0xF))  << 32) |
								 (((u64)(raw_timestamp->ts_nibbles[1]&0xF))  << 28) |
								 (((u64)(raw_timestamp->ts_nibbles[2]&0xF))  << 24) | 
                                 (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 20) |
                                 (((u64)(raw_timestamp->ts_nibbles[4]&0xF))  << 16) |
                                 (((u64)(raw_timestamp->ts_nibbles[5]&0xF))  << 12) |
                                 (((u64)(raw_timestamp->ts_nibbles[6]&0xF))  <<  8) |
                                 (((u64)(raw_timestamp->ts_nibbles[7]&0xF))  <<  4) |
                                  ((u64)(raw_timestamp->ts_nibbles[8]&0xF)));					
                    break;
                }
                case 10:
                {
				    mask      = 0xFFFFFFFFFFULL;
					new_value &= ~mask;
                    new_value |= ((((u64)(raw_timestamp->ts_nibbles[0]&0xF))  << 36) |
								 (((u64)(raw_timestamp->ts_nibbles[1]&0xF))  << 32) |
								 (((u64)(raw_timestamp->ts_nibbles[2]&0xF))  << 28) | 
                                 (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 24) |
                                 (((u64)(raw_timestamp->ts_nibbles[4]&0xF))  << 20) |
                                 (((u64)(raw_timestamp->ts_nibbles[5]&0xF))  << 16) |
                                 (((u64)(raw_timestamp->ts_nibbles[6]&0xF))  << 12) |
                                 (((u64)(raw_timestamp->ts_nibbles[7]&0xF))  << 8)  |
                                 (((u64)(raw_timestamp->ts_nibbles[8]&0xF))  << 4)  |
                                  ((u64)(raw_timestamp->ts_nibbles[9]&0xF)));
                    break;
                }
                case 11:
                {
				    mask      = 0xFFFFFFFFFFFULL;
					new_value &= ~mask;
                    new_value |= ((((u64)(raw_timestamp->ts_nibbles[0]&0xF))  << 40) | 
							     (((u64)(raw_timestamp->ts_nibbles[1]&0xF))  << 36) | 
								 (((u64)(raw_timestamp->ts_nibbles[2]&0xF))  << 32) | 
				 				 (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 28) | 
                                 (((u64)(raw_timestamp->ts_nibbles[4]&0xF))  << 24) |
                                 (((u64)(raw_timestamp->ts_nibbles[5]&0xF))  << 20) |
                                 (((u64)(raw_timestamp->ts_nibbles[6]&0xF))  << 16) |
                                 (((u64)(raw_timestamp->ts_nibbles[7]&0xF))  << 12) |
                                 (((u64)(raw_timestamp->ts_nibbles[8]&0xF))  << 8)  |
                                 (((u64)(raw_timestamp->ts_nibbles[9]&0xF))  << 4)  |
                                 ((u64)(raw_timestamp->ts_nibbles[10]&0xF)));
                    break;
                }
                case 12:
                {
				    mask      = 0xFFFFFFFFFFFFULL;
					new_value &= ~mask;
                    new_value |= ((((u64)(raw_timestamp->ts_nibbles[0]&0xF))  << 44) |
								 (((u64)(raw_timestamp->ts_nibbles[1]&0xF))  << 40) |
								 (((u64)(raw_timestamp->ts_nibbles[2]&0xF))  << 36) |
								 (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 32) |
					             (((u64)(raw_timestamp->ts_nibbles[4]&0xF))  << 28) | 
                                 (((u64)(raw_timestamp->ts_nibbles[5]&0xF))  << 24) |
                                 (((u64)(raw_timestamp->ts_nibbles[6]&0xF))  << 20) |
                                 (((u64)(raw_timestamp->ts_nibbles[7]&0xF))  << 16) |
                                 (((u64)(raw_timestamp->ts_nibbles[8]&0xF))  << 12) |
                                 (((u64)(raw_timestamp->ts_nibbles[9]&0xF))  << 8)  |
                                 (((u64)(raw_timestamp->ts_nibbles[10]&0xF)) << 4)  |
                                  ((u64)(raw_timestamp->ts_nibbles[11]&0xF)));
                    break;
                }
                case 14:
                {
				    mask      = 0xFFFFFFFFFFFFFFULL;
					new_value &= ~mask;
                    new_value |= ((((u64)(raw_timestamp->ts_nibbles[0]&0xF))  << 52) |
					             (((u64)(raw_timestamp->ts_nibbles[1]&0xF))  << 48) |
					             (((u64)(raw_timestamp->ts_nibbles[2]&0xF))  << 44) |
					             (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 40) |
								 (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 36) |
								 (((u64)(raw_timestamp->ts_nibbles[5]&0xF))  << 32) |
								 (((u64)(raw_timestamp->ts_nibbles[6]&0xF))  << 28) | 
                                 (((u64)(raw_timestamp->ts_nibbles[7]&0xF))  << 24) |
                                 (((u64)(raw_timestamp->ts_nibbles[8]&0xF))  << 20) |
                                 (((u64)(raw_timestamp->ts_nibbles[9]&0xF))  << 16) |
                                 (((u64)(raw_timestamp->ts_nibbles[10]&0xF)) << 12) |
                                 (((u64)(raw_timestamp->ts_nibbles[11]&0xF)) << 8)  |
                                 (((u64)(raw_timestamp->ts_nibbles[12]&0xF)) << 4)  |
                                 ((u64)(raw_timestamp->ts_nibbles[13]&0xF)));
                    break;
                }
                case 16:
                {
				    mask      = 0xFFFFFFFFFFFFFFFFULL;
					new_value &= ~mask;
				    new_value |= ((((u64)(raw_timestamp->ts_nibbles[0]&0xF))  << 60) |
					             (((u64)(raw_timestamp->ts_nibbles[1]&0xF))  << 56) |
					             (((u64)(raw_timestamp->ts_nibbles[2]&0xF))  << 52) |
					             (((u64)(raw_timestamp->ts_nibbles[3]&0xF))  << 48) |
					             (((u64)(raw_timestamp->ts_nibbles[4]&0xF))  << 44) |
					             (((u64)(raw_timestamp->ts_nibbles[5]&0xF))  << 40) |
								 (((u64)(raw_timestamp->ts_nibbles[6]&0xF))  << 36) |
								 (((u64)(raw_timestamp->ts_nibbles[7]&0xF))  << 32) |
								 (((u64)(raw_timestamp->ts_nibbles[8]&0xF))  << 28) | 
                                 (((u64)(raw_timestamp->ts_nibbles[9]&0xF))  << 24) |
                                 (((u64)(raw_timestamp->ts_nibbles[10]&0xF)) << 20) |
                                 (((u64)(raw_timestamp->ts_nibbles[11]&0xF)) << 16) |
                                 (((u64)(raw_timestamp->ts_nibbles[12]&0xF)) << 12) |
                                 (((u64)(raw_timestamp->ts_nibbles[13]&0xF)) << 8)  |
                                 (((u64)(raw_timestamp->ts_nibbles[14]&0xF)) << 4)  |
                                  ((u64)(raw_timestamp->ts_nibbles[15]&0xF)));
                    
                    break;
                }
                default:
                {
                    return stp2_dec_err_internal_decoder_error;
                }
            } /* switch (raw_timestamp->ts_nibble_length) */
            break;
       } /* Case STP2_VERSION_V2_NAT */
       default:
       {
            return stp2_dec_err_currently_unsupported_timestamp_format;
       }

    } /* switch (version) */

    switch (version)
    {
        case STP2_VERSION_V2_NATDELTA:
		{
			//Formula:
			// LTS Cycles = (LTS Value +1) 2^G; where G is granulariy factor and G <= 6
			//G=0 -> 2^G = 1;
			//G=1 -> 2^G = 2;
			//G=2 -> 2^G = 4;
			//G=3 -> 2^G = 8;
			// LTS Cycles = (LTS Value +1) 4^(G-3) when G > 6 and LTS Value < 255
			// LTS Cycles = 2^32 when G = 15 and LTS Value = 255
			unsigned int nFactor = 2;
			if( current_granularity ==0 )
			{
				nFactor =1;
			}
			//The case of using the formula: 2^G
			else if( 6 >= current_granularity )
			{
				for( unsigned int i=1; i<current_granularity; i++ )
					nFactor *= 2;
			}
			// The case of nTSTotal = 2^32
			else if( ( 15 == current_granularity ) && ( 255 == new_value ) )
			{
				for( unsigned int i=1; i<32; i++ )
					*new_time *= 2;
				return stp2_dec_success;
			}
			//The case of using the formulat: 4^(G-3)
			else 
			{
				for( unsigned int i=1; i<(unsigned int)( current_granularity-3 ); i++ )
					nFactor *= 4;

				nFactor *= 2;
			}
			new_value = nFactor * ( new_value + 1 );


			*new_time  = ( current_time + new_value );
			break;
		}
        case STP2_VERSION_V2_NAT:
		{
		    time_tp   = current_time & ~mask;
			time_tp  |= new_value;
			*new_time  = time_tp;
			break;
		}
	    case STP2_VERSION_V2_GRAY:
		{
			u64 nNew_gray = gray_to_bin( new_value );
			*new_time = nNew_gray;
		}
		default:
		{
			return stp2_dec_err_currently_unsupported_timestamp_format;
		}
	}
    return stp2_dec_success;
}

u64 bin_to_gray( const u64 nlast_timestamp )
{
	u64 nGray_value = 0;
	nGray_value = ( nlast_timestamp & 0x8000000000000000ULL );
	for( int i=62; i>=0; i-- )
	{
		u64 nMask = ( 0x4000000000000000ULL >> ( 62 - i ) );
		nGray_value &= ~nMask;
		u64 nTmp1 = nlast_timestamp & nMask;
		u64 nTmp2 = nlast_timestamp & ( nMask << 1 );
		nGray_value |= ( nTmp1^( nTmp2 >> 1 ) );
	}
	return nGray_value;
}

u64 gray_to_bin( const u64 nnew_gray )
{
	u64 nBin_value=0;
	for( unsigned int nBin_bit=0; nBin_bit<64; nBin_bit++ )
	{
		u64 nBitMask = 0x1;
		u64 nTmp1 = ( nnew_gray & ( nBitMask << nBin_bit ) ) >> nBin_bit;
		for( unsigned int nGray_bit=nBin_bit+1; nGray_bit<64; nGray_bit++ )
		{
			nBitMask = 0x1;
			u64 nTmp2 = ( nnew_gray & ( nBitMask << nGray_bit ) );
			nTmp2 = ( nTmp2 >> nGray_bit );
			nTmp1 = nTmp1 ^ nTmp2;
		}
		nBin_value |= ( nTmp1 << nBin_bit );
	}
	return nBin_value;
}


/* EOF */

