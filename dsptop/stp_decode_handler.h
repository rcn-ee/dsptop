/*
 * stp_decode_handler.h
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

#ifndef STP_DECODE_HANDLER_H
#define STP_DECODE_HANDLER_H

#include <stdlib.h>
#include "tiulm.h"
#include "dsptop.h"

typedef struct {
    uint8_t master_id;
    int ulm_pk_type;    /* Can be a ulm_state_t, ulm_mem_t, ulm_temp_t, or stm_msg */
    int ulm_usage;      /* ULM_STATE_IDLE, ULM_STATE_RUN or -1 if not a state msg */
    struct timeval tv;  /* Timestamp in timeval format */
    bool seq_id_valid;   
    uint8_t seq_id;
    bool value_1_valid;
    bool value_2_valid;
    uint32_t value_1;   
    uint32_t value_2;
}stp_ulm_t;

void stp_decode_open(ulm_dcm_info_t * dcm_info_p, dsp_type_t * dsp_p);
void stp_decode_close();
void stp_decode_strip_twp(uint32_t * etb_p, size_t etb_size, 
                          uint8_t * etb_twpstrip_p, size_t * etb_twpstrip_size_p);
void stp_decode_etb2ulm(uint8_t * etb_twpstrip_p, size_t etb_twpstrip_size,
                        uint32_t etb_twpstrip_startbyte_pos, 
                        uint32_t * etb_twpstrip_nextbyte_pos, 
                        bool * etb_twpstrip_allbytes_processed, 
                        stp_ulm_t ** ulm_msg_p, int * ulm_msg_cnt);
void stp_decode_log2file(stp_ulm_t * ulm_msg_p, int ulm_msg_cnt, char * log_filename_p,
                         uint32_t max_filesize, dsp_type_t * dsp, bool append, struct timeval * offset_tv);
void stp_decode_memvalue(uint32_t mem_value, int * num_1KB_blocks, double * percent_free);

#endif /* STP_DECODE_HANDLER_H */
