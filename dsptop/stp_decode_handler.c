/*
 * stp_decode_handler.c
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

/* These macros, when set to 1, include additional logging. 
 * Warning: setting these when dsps are generating a high ulm message rate
 * can cause dsptop to hang.
 */
#define LOG_TWP 0

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include "stp_decode_handler.h"
#include "stpv2_dec.h"
#include "tiulm.h"
#include "error_handler.h"
#include "error.h"
#include "debug_log.h"
#include "dsptop.h"
#include "utility.h"

/******************************************************************************
 * Private Declarations
 *****************************************************************************/
static double stm_period = 0;
static stp2_dec_device_ptr dec_handle_ptr = NULL;
static uint8_t atb_id;
static int num_stm_masters = 0;
static dsp_type_t * config_dsp_p = NULL;

typedef enum {
    MSG_TRACK_COMPLETE,
    MSG_TRACK_D32_NEXT,
    MSG_TRACK_D32TS_NEXT,
    ERR_PKT2,
    ERR_PKT3
} statemsg_track_t;

struct incomplete_statemsg_t {
    stp_ulm_t ulm_msg;
    statemsg_track_t statemsg_track; 
};

/******************************************************************************
 * Private Function Prototypes
 *****************************************************************************/
static int stp_decode_get_msgcnt(struct stp2_packet * packet_buf_p,  uint32_t decode_num_packets);
static void stp_decoder_stp_to_ulmmsg(struct stp2_packet * packet_buf_p,  uint32_t decode_num_packets,
                                      stp_ulm_t * ulm_msg_p, int num_ulm_msgs);

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 * stp_decode_open
 *
 *  - Open the decoder
 *    
 *****************************************************************************/
void stp_decode_open(ulm_dcm_info_t * dcm_info_p, dsp_type_t * dsp_p)
{
    LOGFUNC();

    /* Check that the device is compatible */
    if ((dcm_info_p->stp_version != 2) || (dcm_info_p->using_twp == false)) {
        char * msg = "Trace data format error (not stpv2 or not twp)";
        LOGMSG("%s:stpv2 version is %d, using_twp is %d", __func__, dcm_info_p->stp_version, dcm_info_p->using_twp);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    } else {

        enum return_code_stp2_dec dec_ret;

        /* If the decoder is already open, close it */
        if (dec_handle_ptr != NULL) {
            stp_decode_close();
        }

        /* Open the decoder */
        dec_ret = stp2_dec_open(&dec_handle_ptr);
     
        if (dec_ret != stp2_dec_success) {
            char * msg = "Decoder open error";
            LOGMSG("%s:%s %d", __func__, msg, dec_ret);
            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
        }

        /* Set statics */
        stm_period = 1/(double)util_get_freq(DEBUGSS_STM_CLK);

        atb_id = dcm_info_p->atb_id;

        config_dsp_p = dsp_p;
        num_stm_masters = dsp_p->num_dsps;

        LOGMSG("%s: STM Freq is: %f Hz", __func__, 1/stm_period);
        LOGMSG("%s: Decoder opened successfully", __func__);

    }
}

/******************************************************************************
 * stp_decode_close
 *
 *  - Close the decoder
 *    
 *****************************************************************************/
void stp_decode_close()
{
    LOGFUNC();

    enum return_code_stp2_dec dec_ret = stp2_dec_close(dec_handle_ptr);
    if (dec_ret != stp2_dec_success) {
        char * msg = "Decoder close error";
        LOGMSG("%s:%s %d", __func__, msg, dec_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

    dec_handle_ptr = NULL;

    LOGMSG("%s: Decoder close successfully", __func__);

}



/******************************************************************************
 * stp_decode_etb2ulm
 *
 *  - Decode ulm stpv2 messages from a etb source
 *    
 *****************************************************************************/
void stp_decode_etb2ulm(uint8_t * etb_twpstrip_p, size_t etb_twpstrip_size,
                        uint32_t etb_twpstrip_startbyte_pos, 
                        uint32_t * etb_twpstrip_nextbyte_pos, 
                        bool * etb_twpstrip_allbytes_processed, 
                        stp_ulm_t ** ulm_msg_p, int * ulm_msg_cnt_p)
{
    LOGFUNC();

    enum return_code_stp2_dec dec_ret;

    /* Calculate the max number of stpv2 packets. 
     * Note: 6 nibbles is the size of the smallest ulm message (a D8TS stpv2 packet)
     *       less the master and channel nibbles which are optional.
     *       So this will yield the max number of stpv2 packets possible.  
     */
    const int stp2_min_nibbles_per_packet = 6;
    int stp2_max_packets = etb_twpstrip_size/stp2_min_nibbles_per_packet;

    LOGMSG("%s: stp2_max_packets is %d, size of stp2_packet_buf will be %d",
           __func__, stp2_max_packets, sizeof(struct stp2_packet) * stp2_max_packets);

    /* Input to decoder */
    uint32_t decode_startbyte_pos = 0;
    enum stp2_nibble_pos decode_startnibble_pos = STP2_NIBBLE_POS_LOWER;    
    bool decode_sync_detection = (etb_twpstrip_startbyte_pos > 0) ? false : true;

    /* Output from decoder */
    uint32_t decode_num_packets_total = 0;
                                
    /* Output from decoder - in case the input buffer is not fully decoded */
    BOOL decode_allbytes_processed = false;
    BOOL decode_lastpacket_partial = false;
    uint32_t decode_nextbyte_pos = etb_twpstrip_startbyte_pos;   
    enum stp2_nibble_pos decode_nextnibble_pos = STP2_NIBBLE_POS_LOWER;

    int decode_loopcnt = 0;

    int free_packets = stp2_max_packets;
    
    struct stp2_packet stp2_packet_buf[stp2_max_packets];
    struct stp2_packet * packet_buf_p = stp2_packet_buf;

    do {

        /* Decode */
        uint32_t decode_num_packets = 0;
        decode_startbyte_pos = decode_nextbyte_pos;
        decode_startnibble_pos = decode_nextnibble_pos;      


        dec_ret = stp2_dec_decode(dec_handle_ptr, etb_twpstrip_p, etb_twpstrip_size, 
                                  decode_startbyte_pos, decode_startnibble_pos, free_packets, decode_sync_detection,
                                  packet_buf_p,  &decode_num_packets, &decode_nextbyte_pos, &decode_nextnibble_pos,
                                  &decode_allbytes_processed, 
                                  &decode_lastpacket_partial );

        LOGMSG2("%s:Decode loop cnt is %d", __func__, decode_loopcnt++);
        LOGMSG2("%s:Decoded %d stpv2 packets", __func__, decode_num_packets);
        LOGMSG2("%s:Decode all bytes %d", __func__, decode_allbytes_processed);
        LOGMSG2("%s:Decode last packet partial %d", __func__, decode_lastpacket_partial);
        LOGMSG2("%s:Decode next byte position %d", __func__, decode_nextbyte_pos);
        LOGMSG2("%s:Decode next nibble position %d", __func__, decode_nextnibble_pos); 

        decode_sync_detection = false;

        switch(dec_ret) {
            case stp2_dec_success:
                break;
            case stp2_dec_err_lost_sync:
                /* This error typically means the decoder found a invalid sync
                 * (for stpv2 this means not 21 0xF's), but in reality a valid 
                 * packet may have been prematurely terminated by a valid sync.
                 * So restart decoding, but detect a sync first.
                 */
                decode_sync_detection = true;
                LOGMSG2("%s:Decode retry due to lost sync", __func__);
                dec_ret = stp2_dec_success;
                break;
            case stp2_dec_err_internal_decoder_error:
                /* Decoder found something it could not decode,
                 * so try again from the next sync.
                 */
                decode_sync_detection = true;
                LOGMSG2("%s:Decode retry due to invalid protocol detected", __func__);
                dec_ret = stp2_dec_success;
                break;
            case stp2_dec_err_null_parameter:
            case stp2_dec_err_currently_unsupported_timestamp_format:
            case stp2_dec_err_memory_alloc_failure:
            default:
            {
                char * msg = "Decoder error";
                LOGMSG2("%s:%s %d", __func__, msg, dec_ret);
                err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            } 
        } 
        
        if (decode_allbytes_processed && decode_lastpacket_partial) {
            LOGMSG2("%s:Decoder found a partial packet at the end of the data", __func__);
        }               
    
        free_packets -= decode_num_packets;
        decode_num_packets_total += decode_num_packets;
        packet_buf_p += decode_num_packets;

        /* If any data left to decode continue */
    } while ((decode_allbytes_processed == false) && (free_packets > 0));

    *etb_twpstrip_nextbyte_pos = decode_nextbyte_pos;
    *etb_twpstrip_allbytes_processed = decode_allbytes_processed;

    /* Set the packet_buf_p back to the beginning of the buffer */
    packet_buf_p = stp2_packet_buf;

    /*Convert stp to ulm messages */

    int num_ulm_msgs = stp_decode_get_msgcnt(packet_buf_p, decode_num_packets_total);
    LOGMSG("%s: Found %d ulm packets out of %d stp packets", __func__, num_ulm_msgs, decode_num_packets_total);

    /* Note: If num_ulm_msgs is 0 (which is not typical), will still call malloc which will provide a pointer
     * that can be freed later.
     */ 

    *ulm_msg_p = malloc(sizeof(stp_ulm_t) * num_ulm_msgs);
    if (*ulm_msg_p == NULL) {
        err_handler(ERR_TYPE_LOCAL, ERR_MEM_ALLOC, NULL);
        /* Dummy return to keep code checker from declaring
         * an issue, err_handler will call exit.
         */
        return;
    }

    stp_decoder_stp_to_ulmmsg(packet_buf_p, decode_num_packets_total, *ulm_msg_p, num_ulm_msgs);

    *ulm_msg_cnt_p = num_ulm_msgs;

}

/******************************************************************************
 * stp_decode_strip_twp
 *
 *  - Strip twp formatting from etb data
 *  - Only retain etb data with a matching atb id 
 *  - Assumes etb_twpstrip_p is the same size in bytes as etb_size
 * 
 *****************************************************************************/
void stp_decode_strip_twp(uint32_t * etb_p, size_t etb_size, 
                          uint8_t * etb_twpstrip_p, size_t * etb_twpstrip_size_p)
{
    const uint32_t twp_frame_bytecnt = 16;
    const unsigned int twp_frame_wordcnt = 8;
    const unsigned int twp_frame_flag_index = 7;
    const unsigned int twp_frame_flag_shift = 8;
    const uint16_t twp_frame_newid_mask = 1;
    const uint16_t twp_frame_newid_shift = 1;
    const uint16_t twp_frame_atbid_mask = 0x7f;
    const uint16_t twp_newid_nextword = 1;

    *etb_twpstrip_size_p = 0;

    /* With twp wrapped data we expect etb_size_bytes to be a whole number of twp frames.
     * In case there is not a whole number of twp frames, will provide what we can
     * and throw away the remainder.
     */
    if (etb_size % twp_frame_bytecnt) {
        LOGMSG("%s:Warning - etb buffer does not contain a whole number of twp frames", __func__);
    }

    int twp_fram_cnt = etb_size/twp_frame_bytecnt;

    LOGMSG2("%s:number of etb bytes is %d, number of twp frames is %d", __func__, etb_size, twp_fram_cnt);

    /* Can start a frame with data, so the last id may still be valid */
    bool prev_atbid_valid = false;
    bool curr_atbid_valid = false; 
    bool apply_nextbyte_curr = false;
   
    uint16_t * twp_fram_p = (uint16_t *)etb_p; 
    uint8_t * etb_twpstrip_lp = etb_twpstrip_p;

    int pkt_cnt = 0;

    do {

        uint16_t new_id;
        uint16_t twp_frame_flags = twp_fram_p[twp_frame_flag_index] >> twp_frame_flag_shift;

#if LOG_TWP
        LOGMSG2("%s: twp frame flags is 0x%02.2x", __func__, twp_frame_flags);
        for (int twp_frame_index = 0; twp_frame_index < twp_frame_wordcnt; twp_frame_index++) {
            LOGMSG2("%s: twp frame %d is: 0x%04.4x", __func__, twp_frame_index, twp_fram_p[twp_frame_index]);
        }
#endif
        for (int twp_frame_index = 0; twp_frame_index < twp_frame_wordcnt; twp_frame_index++) {

            uint16_t twp_word = twp_fram_p[twp_frame_index];

            /* Test the F bit */
            if (twp_word & twp_frame_newid_mask) {
                /* If a new id then figure out how to disposition the next byte */
                new_id = (twp_word >> twp_frame_newid_shift) & twp_frame_atbid_mask;
                
                prev_atbid_valid = curr_atbid_valid;                
                curr_atbid_valid = (new_id == atb_id) ? true : false;
                apply_nextbyte_curr = ((twp_frame_flags >> twp_frame_index) & twp_newid_nextword) ? false : true;
#if LOG_TWP
                LOGMSG2("%s: ID in frame %d", __func__, twp_frame_index);
                LOGMSG2("%s: prev_atbid_valid is %d", __func__, prev_atbid_valid);
                LOGMSG2("%s: curr_atbid_valid is %d", __func__, curr_atbid_valid);
                LOGMSG2("%s: apply_nextbyte_curr is %d", __func__, apply_nextbyte_curr);
#endif


                /* Disposition the valid byte if not frame index 7 */
                if (twp_frame_index != 7) {

                    if (   (prev_atbid_valid && !apply_nextbyte_curr)
                        || (curr_atbid_valid && apply_nextbyte_curr)) {
#if LOG_TWP
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p, (uint8_t)((twp_word >> 8) & 0x000f));
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p + 1, (uint8_t)(twp_word >> 12));
#endif
                        /*must unpack the stp data such that each stp element (4-bits) occupies a byte */
                        *etb_twpstrip_lp++ = (uint8_t)((twp_word >> 8) & 0x000f); 
                        *etb_twpstrip_lp++ = (uint8_t)(twp_word >> 12);
                        *etb_twpstrip_size_p +=2;
                    }
                } 
            } else {
                /* disposition the current word */
                if (twp_frame_index != 7) {
                    if (curr_atbid_valid) {
                        twp_word |= (twp_frame_flags >> twp_frame_index) & 1;
#if LOG_TWP
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p, (uint8_t)(twp_word  & 0x000f));
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p + 1, (uint8_t)((twp_word >> 4) & 0x000f));
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p + 2, (uint8_t)((twp_word >> 8) & 0x000f));
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p + 3, (uint8_t)(twp_word >> 12));
#endif
                        *etb_twpstrip_lp++ = (uint8_t)(twp_word  & 0x000f); 
                        *etb_twpstrip_lp++ = (uint8_t)((twp_word >> 4) & 0x000f); 
                        *etb_twpstrip_lp++ = (uint8_t)((twp_word >> 8) & 0x000f);
                        *etb_twpstrip_lp++ = (uint8_t)(twp_word >> 12);
                        *etb_twpstrip_size_p +=4;

                    }
                } else {
                    if (curr_atbid_valid) {
                        twp_word |= (twp_frame_flags >> twp_frame_index) & 1;
#if LOG_TWP
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p, (uint8_t)(twp_word  & 0x000f));
                        LOGMSG2("%s: stp byte %d 0x%02.2x", __func__, *etb_twpstrip_size_p + 1, (uint8_t)((twp_word >> 4) & 0x000f));
#endif

                        *etb_twpstrip_lp++ = (uint8_t)(twp_word  & 0x000f); 
                        *etb_twpstrip_lp++ = (uint8_t)((twp_word >> 4) & 0x000f); 
                        *etb_twpstrip_size_p +=2;
                    }
                }
            }  
        } /* End of for */

        twp_fram_p +=  twp_frame_wordcnt;

        pkt_cnt++;      

    } while (--twp_fram_cnt != 0);
 
    LOGMSG2("%s:twp packet count is %d", __func__, pkt_cnt);
   
    LOGMSG2("%s:twp stripped etb buffer size in bytes is %d", __func__, *etb_twpstrip_size_p); 

}

/******************************************************************************
 * stp_decode_log2file
 *
 *  - Perform final decode of ulm messages and log to a file
 *  - Format for state messages is:
 *    Elapsed Time,   Delta Time,     Msg  
 *    HH:MM:SEC:MSEC, HH:MM:SEC:MSEC, Idle 
 * 
 *  - Format for mem messages is:
 *    Elapsed Time,   Delta Time,     Msg 
 *    HH:MM:SEC:MSEC, HH:MM:SEC:MSEC, External Code/Data Total:# KB, #.#% Free #.#% Used
 *    HH:MM:SEC:MSEC, HH:MM:SEC:MSEC, External Data Total:# KB, #.#% Free #.#% Used
 *    HH:MM:SEC:MSEC, HH:MM:SEC:MSEC, Internal Data Total:# KB, #.#% Free #.#% Used  
 * 
 *  - Format for temperature messages is:
 *    Elapsed Time,   Delta Time,     msg    
 *    HH:MM:SEC:MSEC, HH:MM:SEC:MSEC, Temperature is ##C   
 *****************************************************************************/
void stp_decode_log2file(stp_ulm_t * ulm_msg_p, int ulm_msg_cnt, char * out_filename_p,
                         uint32_t max_filesize, dsp_type_t * dsp_info_p, bool append, struct timeval * offset_tv)
{
    LOGFUNC();

    uint32_t file_size = 0;
    uint32_t file_write_cnt = 0;
    bool check_max_filesize = false;
    FILE * log_fp = stdout;    

    if (ulm_msg_cnt == 0) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    const char * file_mode = (append) ? "a" : "w";

    /* Open the log file for writing if it's not stdout */
    if (strncmp(out_filename_p, "stdout", 6)) {

        log_fp = fopen(out_filename_p, file_mode);
        if (log_fp == NULL) {
            char * msg = "Can not open log file for writing";
            LOGMSG("%s:%s: %s", __func__, msg, out_filename_p);
            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            /* Dummy return to keep code checker from declaring
             * an issue, err_handler will call exit.
             */
            return;
        }

        /* Initialize filesize_cnt */
        file_size = (append) ? util_get_filesize(out_filename_p) : 0;

        if (max_filesize != 0) {
            check_max_filesize = true;
        }
        
    } else {
        out_filename_p = "stdout";
    }
    LOGMSG("%s: Logging to: %s", __func__, out_filename_p);

    struct timeval delta_tv = {0,0};
    struct timeval current_tv;
    int ulm_index = 0;

    size_t msg_size_max = 1024;
    char msg_buf[msg_size_max];

    /* This is to fix for a Coverity initialization defect */
    msg_buf[0] = '\0';
    
    do {
        size_t fwrite_cnt;
        size_t chr_cnt = 0;

        /* Add the timestamp offset to the current sample */
        ulm_msg_p[ulm_index].tv.tv_sec += offset_tv->tv_sec;
        ulm_msg_p[ulm_index].tv.tv_usec += offset_tv->tv_usec;
        if (ulm_msg_p[ulm_index].tv.tv_usec >= 1000000) {
            ulm_msg_p[ulm_index].tv.tv_usec -= 1000000;
            ulm_msg_p[ulm_index].tv.tv_sec++;
        }

        if (ulm_index != 0) {
            if (ulm_msg_p[ulm_index-1].tv.tv_usec > ulm_msg_p[ulm_index].tv.tv_usec) {
                current_tv.tv_usec = ulm_msg_p[ulm_index].tv.tv_usec + 1000000;
                current_tv.tv_sec = ulm_msg_p[ulm_index].tv.tv_sec - 1;
                
            } else {
                current_tv.tv_usec = ulm_msg_p[ulm_index].tv.tv_usec;
                current_tv.tv_sec = ulm_msg_p[ulm_index].tv.tv_sec;
            }

            delta_tv.tv_usec = current_tv.tv_usec - ulm_msg_p[ulm_index-1].tv.tv_usec;
            delta_tv.tv_sec = current_tv.tv_sec - ulm_msg_p[ulm_index-1].tv.tv_sec;
        } else {
            if (offset_tv->tv_usec > ulm_msg_p[ulm_index].tv.tv_usec) {
                current_tv.tv_usec = ulm_msg_p[ulm_index].tv.tv_usec + 1000000;
                current_tv.tv_sec = ulm_msg_p[ulm_index].tv.tv_sec - 1;
                
            } else {
                current_tv.tv_usec = ulm_msg_p[ulm_index].tv.tv_usec;
                current_tv.tv_sec = ulm_msg_p[ulm_index].tv.tv_sec;
            }

            delta_tv.tv_usec = current_tv.tv_usec - offset_tv->tv_usec;
            delta_tv.tv_sec = current_tv.tv_sec - offset_tv->tv_sec;
        }

        /* HH:MM:SEC */
        chr_cnt = util_get_timestring(msg_buf, msg_size_max, &ulm_msg_p[ulm_index].tv, TIMESTRING_MICROSECOND);
        chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, " ");
        chr_cnt += util_get_timestring(msg_buf + chr_cnt, msg_size_max - chr_cnt, &delta_tv, TIMESTRING_MICROSECOND);

        //Search the master_id table for an id match, then use the index to get the master name.
        char * master_id_p = "id err";
        int i;
        for (i = 0; i < dsp_info_p->num_masters; i++) {
            if (ulm_msg_p[ulm_index].master_id == dsp_info_p->master_id[i]) {
                master_id_p = dsp_info_p->master_names[i];
                break;
            }
        }
        chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, " %s, ", master_id_p);

        const char * message;
        if (  ( ulm_msg_p[ulm_index].ulm_pk_type < ULM_STATE_LAST)
           && ( ulm_msg_p[ulm_index].value_1_valid == false) ){

            message = ulm_get_statemsg(ulm_msg_p[ulm_index].ulm_pk_type);
            chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, "%s", message);

        } else if (  ( ulm_msg_p[ulm_index].ulm_pk_type < ULM_STATE_LAST)
                  && ( ulm_msg_p[ulm_index].value_1_valid == true) ){

            const char * format = ulm_get_statefmt(ulm_msg_p[ulm_index].ulm_pk_type);
            if (format == NULL) {
                char * msg = "Invalid ulm packet type";
                LOGMSG("%s:%s: %d", __func__, msg, ulm_msg_p[ulm_index].ulm_pk_type);
                err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);                
            }
            chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, format, 
                                ulm_msg_p[ulm_index].value_1, ulm_msg_p[ulm_index].value_2);

        } else if (  ( ulm_msg_p[ulm_index].ulm_pk_type >= ULM_STATE_LAST)
                  && ( ulm_msg_p[ulm_index].ulm_pk_type < ULM_MEM_LAST) ){
            switch (ulm_msg_p[ulm_index].ulm_pk_type) {
                case ULM_MEM_EX_CODE_AND_DATA:
                    message = "External Code/Data";
                    break;
                case ULM_MEM_EX_DATA_ONLY:
                    message = "External Data";
                    break;
                case ULM_MEM_IN_DATA_ONLY:
                    message = "Internal Data";
                    break;
                default:
                /* Keep the compiler from generating a warning */
                    message = "Unknown";
                    break;
            }

            int num_1KB_blocks = 0;
            double percent_free = 0;
            stp_decode_memvalue(ulm_msg_p[ulm_index].value_1, &num_1KB_blocks, &percent_free);

            chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, 
                                "%s Total %d KB, %2.1f%% Free, %2.1f%% Used", 
                                message, num_1KB_blocks, percent_free, 100.0 - percent_free);

        } else if (ulm_msg_p[ulm_index].ulm_pk_type == ULM_TEMP) {

            int8_t temp = ulm_msg_p[ulm_index].value_1;
            chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, 
                                "Device Temperature %d C", temp);

        } else if (ulm_msg_p[ulm_index].ulm_pk_type == ULM_SYNC) {

            chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, "Sync");

        } else {
            chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, 
                                "Non-supported message - packet type is %d",
                                ulm_msg_p[ulm_index].ulm_pk_type);
        }
           
        LOGMSG2("%s:%s: chr_cnt is %d", __func__, msg_buf, chr_cnt);

        chr_cnt += snprintf(msg_buf + chr_cnt, msg_size_max - chr_cnt, "\n");

        file_write_cnt += chr_cnt;
        if ((check_max_filesize) && ((file_size + file_write_cnt) > max_filesize)) {

            fclose(log_fp);

            char * msg = "Log file exceeded max file size - see -m";
            LOGMSG("%s:%s: %d", __func__, msg, file_size + chr_cnt);
            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);            
        }
        
        fwrite_cnt = fwrite(msg_buf, 1, chr_cnt, log_fp);
        if (fwrite_cnt != chr_cnt) {
            char * msg = "Error while writing log file";
            LOGMSG("%s:%s: %d", __func__, msg, fwrite_cnt);
            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
        }

        ulm_index++;

    } while(--ulm_msg_cnt != 0);

    /* Just in case using stdout because we can't close */
    if (fflush(log_fp) < 0) {
        err_handler(ERR_TYPE_SYSTEM, ERR_LOGFILE, NULL);
    }

    if ((log_fp != stdout) && (log_fp != NULL)) {
        fclose(log_fp);
    }

}

/******************************************************************************
 * stp_decode_memmsg
 *
 *****************************************************************************/
void stp_decode_memvalue(uint32_t mem_value, int * num_1KB_blocks, double * percent_free)
{
    *num_1KB_blocks = (mem_value >> 10)*64;
    *percent_free = (double)(mem_value & 0x3FF)/10.0;
}
/******************************************************************************
 * Private Functions
 *****************************************************************************/

/******************************************************************************
 * stp_timestamp_to_tv
 *
 *  - Convert from STM clocks (fclk /3) to real time
 *  - Note: timestamp returned from sptv2 decoder is accumulated time (not delta)
 * 
 *****************************************************************************/
static inline void stp_timestamp_to_tv(struct timeval * tv, uint64_t stp_timestamp)
{   
    double ts = ((double)stp_timestamp) * stm_period;   
    double ts_sec;
    double ts_usec  = modf(ts, &ts_sec) * 1e6;
   
    tv->tv_sec = (long)ts_sec;
    tv->tv_usec = (long)ts_usec;
}

/******************************************************************************
 * stp_d8ts_to_ulm
 *
 * - Convert a d8ts stp packet to a ulm state packet
 * 
 *****************************************************************************/
static inline void stp_d8ts_to_ulm(struct stp2_packet * packet_buf_p, int stp_index, stp_ulm_t * ulm_msg_p, int ulm_index)
{

    ulm_msg_p[ulm_index].master_id = packet_buf_p[stp_index].master_id;
    ulm_msg_p[ulm_index].ulm_pk_type = packet_buf_p[stp_index].channel_id;
    stp_timestamp_to_tv(&ulm_msg_p[ulm_index].tv, packet_buf_p[stp_index].timestamp);
    if (packet_buf_p[stp_index].channel_id < ULM_STATE_LAST) {
        ulm_msg_p[ulm_index].ulm_usage = ulm_get_runstate(packet_buf_p[stp_index].channel_id);
        ulm_msg_p[ulm_index].seq_id_valid = true;
        ulm_msg_p[ulm_index].seq_id = (uint8_t)packet_buf_p[stp_index].payload;
        ulm_msg_p[ulm_index].value_1_valid = false;
        ulm_msg_p[ulm_index].value_2_valid = false;

        LOGMSG2("%s:ulm index %d, stp index %d, master 0x%04.4x, packet type %d, State Message, seq id %4d, time %d.%06d, timestamp 0x%x",
               __func__, ulm_index, stp_index, ulm_msg_p[ulm_index].master_id, ulm_msg_p[ulm_index].ulm_pk_type, 
               ulm_msg_p[ulm_index].seq_id,  ulm_msg_p[ulm_index].tv.tv_sec, ulm_msg_p[ulm_index].tv.tv_usec, packet_buf_p[stp_index].timestamp );

    } else {
        ulm_msg_p[ulm_index].ulm_usage = packet_buf_p[stp_index].channel_id;
        ulm_msg_p[ulm_index].seq_id_valid = false;
        ulm_msg_p[ulm_index].value_1_valid = true;
        ulm_msg_p[ulm_index].value_1 = (uint32_t)packet_buf_p[stp_index].payload;
        ulm_msg_p[ulm_index].value_2_valid = false;

        LOGMSG2("%s:ulm index %d, stp index %d, master 0x%04.4x, packet type %d, Temperature, value 0x%02.2x, time %d.%06d, timestamp 0x%x",
               __func__, ulm_index, stp_index, ulm_msg_p[ulm_index].master_id, ulm_msg_p[ulm_index].ulm_pk_type, 
               ulm_msg_p[ulm_index].value_1,  ulm_msg_p[ulm_index].tv.tv_sec, ulm_msg_p[ulm_index].tv.tv_usec, packet_buf_p[stp_index].timestamp );


    }
}

/******************************************************************************
 * stp_d32ts_to_ulm
 *
 * - Convert a d32ts stp packet to a ulm state packet
 * 
 *****************************************************************************/
static inline void stp_d32ts_to_ulm(struct stp2_packet * packet_buf_p, int stp_index, stp_ulm_t * ulm_msg_p, int ulm_index)
{
    ulm_msg_p[ulm_index].master_id = packet_buf_p[stp_index].master_id;
    ulm_msg_p[ulm_index].ulm_pk_type = packet_buf_p[stp_index].channel_id;
    ulm_msg_p[ulm_index].ulm_usage = packet_buf_p[stp_index].channel_id;
    ulm_msg_p[ulm_index].seq_id_valid = false;
    ulm_msg_p[ulm_index].value_1_valid = true;
    ulm_msg_p[ulm_index].value_1 = (uint32_t)packet_buf_p[stp_index].payload;
    ulm_msg_p[ulm_index].value_2_valid = false;
    stp_timestamp_to_tv(&ulm_msg_p[ulm_index].tv, packet_buf_p[stp_index].timestamp);

    /* ULM_SYNC will set ulm_usage to -1 */
    if (  (ulm_msg_p[ulm_index].ulm_pk_type != ULM_MEM_EX_CODE_AND_DATA)
       && (ulm_msg_p[ulm_index].ulm_pk_type != ULM_MEM_EX_DATA_ONLY)
       && (ulm_msg_p[ulm_index].ulm_pk_type != ULM_MEM_IN_DATA_ONLY) ) {

        ulm_msg_p[ulm_index].ulm_usage = -1;
    }

    if (g_log_enable == true) {
        char * msg;

        switch(ulm_msg_p[ulm_index].ulm_pk_type) {
            case ULM_MEM_EX_CODE_AND_DATA:
                msg = "External memory (code and data)";
                break;
            case ULM_MEM_EX_DATA_ONLY:
                msg = "External memory (data only)";
                break;
            case ULM_MEM_IN_DATA_ONLY:
                msg = "Internal memory (data only)";
                break;
            case ULM_SYNC:
                msg = "Sync message detected";
                break;
            default:
                msg = "Unknown packet";
        }

        LOGMSG2("%s:ulm index %d, stp index %d, master 0x%04.4x, packet type %d, %s, %08.8x, time %d.%06d, timestamp 0x%x",
               __func__, ulm_index, stp_index,  ulm_msg_p[ulm_index].master_id, ulm_msg_p[ulm_index].ulm_pk_type, 
               msg, ulm_msg_p[ulm_index].value_1, 
               ulm_msg_p[ulm_index].tv.tv_sec, ulm_msg_p[ulm_index].tv.tv_usec, packet_buf_p[stp_index].timestamp);
    }

}

/******************************************************************************
 * stp_d8_to_ulm
 *
 * - Convert a d8 stp packet to a ulm state packet
 * 
 *****************************************************************************/
static inline statemsg_track_t stp_d8_to_ulm(struct stp2_packet * packet_buf_p, int stp_index, stp_ulm_t * ulm_msg_p, int ulm_index)
{
    char * msg = "D8 followed by a non-supported stp opcode.";

    ulm_msg_p[ulm_index].master_id = packet_buf_p[stp_index].master_id;
    ulm_msg_p[ulm_index].ulm_pk_type = packet_buf_p[stp_index].channel_id;
    ulm_msg_p[ulm_index].ulm_usage = ulm_get_runstate(packet_buf_p[stp_index].channel_id);
    ulm_msg_p[ulm_index].seq_id_valid = true;
    ulm_msg_p[ulm_index].seq_id = (uint8_t)packet_buf_p[stp_index].payload;

    stp_index++;

    /* If the master id of the next packet does not match the previous, 
     * the next packet is nested, so the current ulm message is incomplete.
     */ 
    if(packet_buf_p[stp_index].master_id != packet_buf_p[stp_index - 1].master_id) {
        return MSG_TRACK_D32_NEXT;
    }

    if (packet_buf_p[stp_index].opcode.sng_nibble == STP2_PKT_TYPE_D32) {
        ulm_msg_p[ulm_index].value_1_valid = true;
        ulm_msg_p[ulm_index].value_1 = (uint32_t)packet_buf_p[stp_index].payload;
    } else {
        LOGMSG2("%s: %s Packet 2 is %d", __func__, msg, packet_buf_p[stp_index].opcode.sng_nibble);
        return ERR_PKT2;
    }

    stp_index++;

    /* If the master id of the next packet does not match the previous, 
     * the next packet is nested, so the current ulm message is incomplete.
     */ 
    if(packet_buf_p[stp_index].master_id != packet_buf_p[stp_index - 1].master_id) {
        return MSG_TRACK_D32TS_NEXT;
    }

    if (packet_buf_p[stp_index].opcode.sng_nibble == STP2_PKT_TYPE_D32MTS) {
        ulm_msg_p[ulm_index].value_2_valid = true;
        ulm_msg_p[ulm_index].value_2 = (uint32_t)packet_buf_p[stp_index].payload;
        stp_timestamp_to_tv(&ulm_msg_p[ulm_index].tv, packet_buf_p[stp_index].timestamp);

    } else {
        LOGMSG2("%s: %s Packet 3 is %d", __func__, msg, packet_buf_p[stp_index].opcode.sng_nibble);
        return ERR_PKT3;
    } 

    LOGMSG2("%s:ulm index %d, stp index %d, master 0x%04.4x, packet type %d, Completed D8 State Message, seq id %4d, value_1 0x%8.8x, value_2 0x%8.8x time %d.%06d, timestamp 0x%llx",
            __func__, ulm_index, stp_index, ulm_msg_p[ulm_index].master_id, ulm_msg_p[ulm_index].ulm_pk_type, 
            ulm_msg_p[ulm_index].seq_id, ulm_msg_p[ulm_index].value_1, ulm_msg_p[ulm_index].value_2, 
            ulm_msg_p[ulm_index].tv.tv_sec, ulm_msg_p[ulm_index].tv.tv_usec, packet_buf_p[stp_index].timestamp );

    return MSG_TRACK_COMPLETE;
}

/******************************************************************************
 * stp_d32_to_statemsg
 *
 * - Restart decoding a incomplete statemsg from a d32 stp packet
 * 
 *****************************************************************************/
static inline statemsg_track_t stp_d32_to_statemsg(struct stp2_packet * packet_buf_p, int stp_index, stp_ulm_t * ulm_msg_p)
{
    char * msg = "Statemsg D32 followed by a non-supported stp opcode.";  

    ulm_msg_p->value_1_valid = true;
    ulm_msg_p->value_1 = (uint32_t)packet_buf_p[stp_index].payload;

    stp_index++;

    /* If the master id of the next packet does not match the previous, 
     * the next packet is nested, so the current ulm message is incomplete.
     */ 
    if(packet_buf_p[stp_index].master_id != packet_buf_p[stp_index - 1].master_id) {
        return MSG_TRACK_D32TS_NEXT;
    }

    if (packet_buf_p[stp_index].opcode.sng_nibble == STP2_PKT_TYPE_D32MTS) {
        ulm_msg_p->value_2_valid = true;
        ulm_msg_p->value_2 = (uint32_t)packet_buf_p[stp_index].payload;
        stp_timestamp_to_tv(&ulm_msg_p->tv, packet_buf_p[stp_index].timestamp);

    } else {
        LOGMSG2("%s: %s Packet 3 is %d", __func__, msg, packet_buf_p[stp_index].opcode.sng_nibble);
        return ERR_PKT2;
    } 

    return MSG_TRACK_COMPLETE;
}

/******************************************************************************
 * stp_d32ts_to_statemsg
 *
 * - Restart decoding a incomplete statemsg from a d32 stp packet
 * 
 *****************************************************************************/
static inline statemsg_track_t stp_d32ts_to_statemsg(struct stp2_packet * packet_buf_p, int stp_index, stp_ulm_t * ulm_msg_p)
{
   
    ulm_msg_p->value_2_valid = true;
    ulm_msg_p->value_2 = (uint32_t)packet_buf_p[stp_index].payload;
    stp_timestamp_to_tv(&ulm_msg_p->tv, packet_buf_p[stp_index].timestamp);

    return MSG_TRACK_COMPLETE;

}

/******************************************************************************
 * stp_decoder_stp_to_ulmmsg
 *
 *  - Convert stp messages to ulm messages
 *
 * Note: If support for standard STM Messages (not just ULM) is ever implemented 
 * (which would be a nice addition to the logging capability), OST decoding must
 * be implemented with support for partial messages (another master interrupts the
 * current message).
 * 
 *****************************************************************************/
static void stp_decoder_stp_to_ulmmsg(struct stp2_packet * packet_buf_p,  uint32_t num_stp_packets,
                                      stp_ulm_t * ulm_msg_p, int num_ulm_msgs)
{

    /* Note: this is the only message type that spans
     * multiple stp packets.
     */
    
    struct incomplete_statemsg_t incomplete_statemsg[num_stm_masters];

    int ulm_index = 0;
    int stp_index = 0;
    statemsg_track_t statemsg_track = MSG_TRACK_COMPLETE;
    char * msg = "Non supported stp opcode found";

    if ((num_ulm_msgs == 0) || (num_stp_packets == 0)) {
        return;
    }

    for (int i = 0; i < num_stm_masters; i++) {
        incomplete_statemsg[i].statemsg_track = MSG_TRACK_COMPLETE;
    }

    LOGMSG2("%s:decoding %d num_ulm_msgs, %d num_stp_packets", __func__, num_ulm_msgs, num_stp_packets);

    do {


        //Convert packet_buf_p[stp_index].master_id to a mid and then use
        // mid to index incomplete_statemsg.
        //mid must be in the range of 0 to num_stm_masters-1.

        uint8_t mid;
        for (mid = 0; mid < config_dsp_p->num_masters; mid++) {
            if (packet_buf_p[stp_index].master_id == config_dsp_p->master_id[mid]) {
                break;
            }
        } 

        switch(packet_buf_p[stp_index].opcodeLength) {
            case 1:
                switch(packet_buf_p[stp_index].opcode.sng_nibble) {

                    case STP2_PKT_TYPE_D8MTS:
                        stp_d8ts_to_ulm(packet_buf_p, stp_index, ulm_msg_p, ulm_index);
                        stp_index++;
                        ulm_index++;
                        break;
                    case STP2_PKT_TYPE_D32MTS:
                    {
                        /* First - check the current master for an incomplete statemsg */
                        stp_ulm_t * incomplete_ulm_msp_p = &incomplete_statemsg[mid].ulm_msg;
                        int incomplete_statemsg_track = incomplete_statemsg[mid].statemsg_track;

                        if (incomplete_statemsg_track == MSG_TRACK_D32TS_NEXT) {

                            statemsg_track = stp_d32ts_to_statemsg(packet_buf_p, stp_index, incomplete_ulm_msp_p);
                            incomplete_statemsg[mid].statemsg_track = MSG_TRACK_COMPLETE;
                            ulm_msg_p[ulm_index] = incomplete_statemsg[mid].ulm_msg;

                            LOGMSG2("%s:ulm index %d, stp index %d, master 0x%04.4x, packet type %d, Completed D32TS State Message, seq id %4d, value_1 0x%8.8x, value_2 0x%8.8x time %d.%06d",
                                __func__, ulm_index, stp_index, ulm_msg_p[ulm_index].master_id, ulm_msg_p[ulm_index].ulm_pk_type, 
                                ulm_msg_p[ulm_index].seq_id, ulm_msg_p[ulm_index].value_1, ulm_msg_p[ulm_index].value_2, 
                                ulm_msg_p[ulm_index].tv.tv_sec, ulm_msg_p[ulm_index].tv.tv_usec);

                            ulm_index++;

                        } else {
                            /* Not part of an incomplete message */
                            stp_d32ts_to_ulm(packet_buf_p, stp_index, ulm_msg_p, ulm_index);

                            /* Filter out sync messages so they have no effect on usage math */
                            if (ulm_msg_p[ulm_index].ulm_pk_type != ULM_SYNC) {
                                ulm_index++;
                            }

                        }
                        stp_index++;
                        break;
                    }
                    case STP2_PKT_TYPE_D8:
                        /* Since this message type spans 3 stp packets, need to test if
                         * at the end of the buffer. If at the end of the buffer,
                         * discard partial messages.
                         */
                        if (stp_index < (num_stp_packets -2)) {
                            statemsg_track = stp_d8_to_ulm(packet_buf_p, stp_index, ulm_msg_p, ulm_index);
                        } else {
                            statemsg_track = MSG_TRACK_COMPLETE;
                            stp_index = num_stp_packets; /* Make sure do loop is terminated */
                            LOGMSG2("%s: Discarded partial state message packet at stp packet %d of %d packets", 
                                   __func__, stp_index, num_stp_packets );   
                        }
                        switch (statemsg_track) {
                            case MSG_TRACK_COMPLETE:
                                stp_index += 3;
                                ulm_index++;
                                break;
                            case MSG_TRACK_D32_NEXT:
                                /* Incomplete D8 message. Save the ulm_msg struct in incomplete_statemsg so it can be put in the
                                 * timestamp order when the end of the message is found.
                                 */
                                incomplete_statemsg[mid].ulm_msg = ulm_msg_p[ulm_index];
                                /* Indicate there is an outstanding message to complete for this master */
                                incomplete_statemsg[mid].statemsg_track = MSG_TRACK_D32_NEXT;
                                /* Only increment the stp_index, and not the ulm_index since the current message was not completed */
                                stp_index += 1;
                                break;
                            case MSG_TRACK_D32TS_NEXT:
                                incomplete_statemsg[mid].ulm_msg = ulm_msg_p[ulm_index];
                                incomplete_statemsg[mid].statemsg_track = MSG_TRACK_D32TS_NEXT;
                                stp_index += 2;
                                break;
                            case ERR_PKT2:
                                /* Error on packet 2, allow next stp message to replace the current partial ulm message */
                                incomplete_statemsg[mid].statemsg_track = MSG_TRACK_COMPLETE;
                                stp_index += 1;
                                break;
                            case ERR_PKT3:
                                /* Error on packet 2, allow next stp message to replace the current partial ulm message */
                                incomplete_statemsg[mid].statemsg_track = MSG_TRACK_COMPLETE;
                                stp_index += 2;
                                break; 
                        }
                        break;
                    case STP2_PKT_TYPE_D32:
                    {
                        /* Check the current master for an incomplete statemsg */
                        stp_ulm_t * incomplete_ulm_msp_p = &incomplete_statemsg[mid].ulm_msg;
                        int incomplete_statemsg_track = incomplete_statemsg[mid].statemsg_track;
                        if (incomplete_statemsg_track == MSG_TRACK_D32_NEXT) {
                            statemsg_track = stp_d32_to_statemsg(packet_buf_p, stp_index, incomplete_ulm_msp_p);

                            switch (statemsg_track) {
                                case MSG_TRACK_COMPLETE:

                                    incomplete_statemsg[mid].statemsg_track = MSG_TRACK_COMPLETE;
                                    ulm_msg_p[ulm_index] = incomplete_statemsg[mid].ulm_msg;

                                    LOGMSG2("%s:ulm index %d, stp index %d, master 0x%04.4x, packet type %d, Completed D32 State Message, seq id %4d, value_1 0x%8.8x, value_2 0x%8.8x time %d.%06d",
                                        __func__, ulm_index, stp_index, ulm_msg_p[ulm_index].master_id, ulm_msg_p[ulm_index].ulm_pk_type, 
                                        ulm_msg_p[ulm_index].seq_id, ulm_msg_p[ulm_index].value_1, ulm_msg_p[ulm_index].value_2, 
                                        ulm_msg_p[ulm_index].tv.tv_sec, ulm_msg_p[ulm_index].tv.tv_usec);

                                    ulm_index++;
                                    stp_index += 2;
                                    break;
                                case MSG_TRACK_D32_NEXT:
                                    /* This can not be returned, just here to keep the compiler from generating a warning */
                                    break;
                                case MSG_TRACK_D32TS_NEXT:
                                    incomplete_statemsg[mid].statemsg_track = MSG_TRACK_D32TS_NEXT;
                                    stp_index += 1;
                                    break;
                                case ERR_PKT2:
                                    /* Error on packet 2, allow next stp message to replace the current partial ulm message */
                                    incomplete_statemsg[mid].statemsg_track = MSG_TRACK_COMPLETE;
                                    stp_index += 1;
                                    break; 
                                case ERR_PKT3:
                                    /* This can not be returned, just here to keep the compiler from generating a warning */
                                    break;
                            }
                            break;
                        } 
                        /* If not a partial message then unexpected stp opcode so fall through to default */
                    }   
                    default:
                        LOGMSG2("%s: Unexpected stp opcode 0x%x - continue", __func__, packet_buf_p[stp_index].opcode.sng_nibble);
                        stp_index++;
                        break;
                }
                break;
            case 2:
#if 0
                switch(packet_buf_p[stp_index].opcode.two_nibble) {
                    case STP2_SEC_NIB_PKT_TYPE_D8TS:
                        stp_d8ts_to_ulm(packet_buf_p, stp_index, ulm_msg_p, ulm_index);
                        stp_index++;
                        break;
                    case STP2_SEC_NIB_PKT_TYPE_D32TS:
                        stp_d32ts_to_ulm(packet_buf_p, stp_index, ulm_msg_p, ulm_index);
                        stp_index++;
                        break;
                    case STP2_SEC_NIB_PKT_TYPE_D8M:
                        stp_d8_to_ulm(packet_buf_p, stp_index, ulm_msg_p, ulm_index);
                        stp_index += 3;;
                        break;
                    default:
                        LOGMSG2("%s: Unexpected two nibble stp opcode 0x%x - continue", __func__, packet_buf_p[stp_index].opcode.sng_nibble);
                        stp_index++;
                        break;
                }
                break;
#endif
            case 3:
            default:
            {

                err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            }
        }

    } while ((ulm_index < num_ulm_msgs) && ( stp_index < num_stp_packets));

}

/******************************************************************************
 * stp_decode_get_msgcnt
 *
 *  - Count number of TS messages in data set
 * 
 *****************************************************************************/
static int stp_decode_get_msgcnt(struct stp2_packet * packet_buf_p,  uint32_t decode_num_packets)
{
    int msg_cnt = 0;
    for (int i = 0; i < decode_num_packets; i++) {
        
        switch(packet_buf_p[i].opcodeLength) {
            case 1:
                switch(packet_buf_p[i].opcode.sng_nibble) {
                    case STP2_PKT_TYPE_D8MTS:
                    case STP2_PKT_TYPE_D16MTS:
                    case STP2_PKT_TYPE_D32MTS:
                    case STP2_PKT_TYPE_D64MTS:
                    case STP2_PKT_TYPE_D4MTS:
                        /* Filter sync packets */
                        if (packet_buf_p[i].channel_id != ULM_SYNC) {
                            msg_cnt++;
                        }
                    default:
                        break;
                }
                break;
            case 2:
#if 0
                switch(packet_buf_p[i].opcode.two_nibble) {
                    case STP2_SEC_NIB_PKT_TYPE_D8TS:
                    case STP2_SEC_NIB_PKT_TYPE_D16TS:
                    case STP2_SEC_NIB_PKT_TYPE_D32TS:
                    case STP2_SEC_NIB_PKT_TYPE_D64TS:
                    case STP2_SEC_NIB_PKT_TYPE_D4TS:
                        /* Filter sync packets */
                        if (packet_buf_p[i].channel_id != ULM_SYNC) {
                            msg_cnt++;
                        }
                    default:
                        break;
                }
                break;
#endif
            case 3:
            default:
            {
                char * msg = "Non supported stp opcode found";
                err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            }
        }
    }

    return msg_cnt;
}

