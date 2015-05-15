/*
 * etb_handler.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "dsptop.h"
#include "error_handler.h"
#include "debug_log.h"
#include "etb_handler.h"
#include "ETBInterface.h"
#include "tiulm.h"
#include "utility.h"

/***************************************************************************** 
 *  Note - The etb_handler is an abstraction layer between dsptop and ETBLib.
 *  
 *****************************************************************************/


/***************************************************************************** 
 *  Definitions, Constants and Statics
 *  
 *****************************************************************************/
static ETBHandle* etb_handle = NULL;

struct etb_queue_t {
    uint32_t * buf_start;       /* Start of the buffer - constant for life of queue element. Used to free the buffer. */
    uint32_t * buf_ptr;         /* Initialized with start of buffer, but is used to schedule partially sent buffers.  */
    int byte_cnt;
    struct etb_queue_t * next;   
};  

static struct etb_queue_t * etb_queue_head = NULL;
static struct etb_queue_t * etb_queue_tail = NULL;
static int etb_queue_cnt = 0;

static size_t etb_size; /* ETB size in bytes*/

static const int max_queue_depth = 1024;

static bool etb_is_enabled = false;
static bool etb_disabled_wrapped = false;
static int etb_words_queued = 0;

/***************************************************************************** 
 *  Internal Function Prototypes 
 *  - See Private Function section below for implementations )
 *****************************************************************************/
static void etb_process_data(int etb_word_cnt);
static int etb_data_available(bool * wrapped);

/***************************************************************************** 
 *  Public Functions
 *  
 *****************************************************************************/

/***************************************************************************** 
 *  etb_config()
 *
 *  Configure ETBLib for the ETB type and operating mode.
 *
 *  - Determine the etb mode and core id.
 *  - Open ETBLib.
 *  
 *****************************************************************************/
size_t etb_config(etb_bufmode_t mode, etb_core_id_t core_id)
{   
    eETB_Error  etb_ret = eETB_Success;
    eETB_Mode etb_mode;
    uint8_t etb_core_id = -1;

    LOGFUNC();

    switch (mode) {
    case ETB_CIRCULAR:
        etb_mode = eETB_TI_Mode;
        break;
    case ETB_FIXED:
        if (core_id == SYS_TIETB) {
            etb_mode = eETB_TI_Mode_AND_Stop_Buffer;
            break;
        }
        char * msg = "ETB only supports circular mode";
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);    
    }

    switch (core_id) {
    case SYS_TIETB:
        etb_core_id = SYS_ETB;
        break;
    case ARM_CSETB:
#ifdef C66AK2Hxx
        etb_core_id = ARM_ETB;
#else
        etb_core_id = SYS_ETB;
#endif
        break;
    default:
        {
            char * msg = "Unreognized core id";
            LOGMSG("%s:%s is %d", __func__, msg, core_id);
            err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, msg);
            /* Dummy return to keep code checker tools from
             * declaring an issue, err_handler will call exit.
             */
            return 0; 
        }       
    }/* End of switch */

    /* Initialize ETB
     *  Note - ETB_open will call cTools_memMap to map the ETB address 
     *  space to the ctoolsprof virtual space.
     */

    LOGMSG("%s:Calling ETB_open, mode is %d, core id is %d", __func__,
            etb_mode, etb_core_id);
 
    ETB_errorCallback etb_err_callback = NULL;   
    etb_ret = ETB_open(etb_err_callback, etb_mode, etb_core_id,
                       &etb_handle, &etb_size);

    etb_size *= 4; /* Convert from words to bytes */

    if (etb_ret != eETB_Success) {
        char * msg = "ETB open failure";
        LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
    } else {
        LOGMSG("%s:ETB_open successful, size of etb is %d bytes", __func__, etb_size);
    }

    return etb_size;

}

/***************************************************************************** 
 *  etb_term()
 *
 *  Close ETBLib.
 * 
 *****************************************************************************/
void etb_term()
{
    eETB_Error  etb_ret;

    LOGFUNC();   
    
    etb_ret  = ETB_close(etb_handle);
    if (etb_ret != eETB_Success) {
        char * msg = "ETB close failure";

        LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
    }

}


/***************************************************************************** 
 *  etb_status()
 *
 *  - Returns the number of words available, if the ETB is enabled, 
 *    and if it has wrapped.
 *  - If etb mode is ONESHOT and recording return number of bytes available, if
 *    not recording then return the number of words queued to transfer.
 *  - If etb mode is DRAIN then always return the number of words currently 
 *    queued.
 * 
 *****************************************************************************/
int etb_status(bool * is_enabled, bool * is_wrapped)
{

    int available_etb_words = 0;

    if (is_wrapped == NULL) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    if (is_enabled != NULL) {
        *is_enabled = etb_is_enabled;
    }

    if (global_state.recording == STATE_UNINITIALIZED) {
        LOGMSG("%s:global_state.recording is STATE_UNINITIALIZED so returning 0", __func__);
        return 0;
    }

    switch (global_state.etb_mode) {
    case ETB_MODE_ONESHOT_FIXED:
    case ETB_MODE_ONESHOT_CIRCULAR:
    {

        /* Get available count from ETB - always need is_wrapped*/
        available_etb_words = etb_data_available(is_wrapped);
        /* If the state is stopped and there is etb data queued, then report queued data */
        if ((global_state.recording == STATE_STOPPED) && (etb_words_queued)) {
                available_etb_words = etb_words_queued;
        } 

        break;
    }
    case ETB_MODE_DRAIN:
    {
        /* Returned data queued */
        available_etb_words = etb_words_queued;
        break;
    }
    /* Needed to fix compiler warning */
    case ETB_MODE_LAST:
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
        return 0;
    }/* End of switch */

    return available_etb_words;
}

/***************************************************************************** 
 *  etb_enable()
 *
 *  - Simply enable the ETB to collect data.
 * 
 *****************************************************************************/
void etb_enable()
{
    eETB_Error  etb_ret;

    LOGFUNC();

    etb_ret = ETB_enable(etb_handle, 0);
    if(etb_ret != eETB_Success) {
        char * msg = "ETB enable failure";

        LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
    }

    etb_is_enabled = true;
    etb_disabled_wrapped = false;

    global_state.recording = STATE_RECORDING;
}

/***************************************************************************** 
 *  etb_disable()
 *
 *  - Simply disable the ETB.
 * 
 *****************************************************************************/
void etb_disable()
{
    eETB_Error  etb_ret;

    LOGFUNC();

    etb_ret = ETB_disable(etb_handle);
    if(etb_ret != eETB_Success) {
        char * msg = "ETB disable failure";

        LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
    }

    /* Need to cache the wrapped status before any ETB_reads. */
    ETBStatus etb_status;
    etb_ret = ETB_status(etb_handle, &etb_status);
    if (etb_ret != eETB_Success) {
        char * msg = "ETB status failure";

        LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
    }
    etb_disabled_wrapped = (etb_status.isWrapped) ? true : false;

    etb_is_enabled = false;

    global_state.recording = STATE_STOPPED;
}

/***************************************************************************** 
 *  etb_add_queue()
 *
 *  This function queues available ETB data (all the heavy lifting is done in 
 *  etb_process_data()).
 * 
 *  - Check that the queue depth has not been exceeded - if so something is really 
 *    wrong. Should only ever be a problem for drain mode.
 *  - If limit is ETB_QUEUE_ALL then all ETB data is read and queued. If limit
 *    is set to ETB_QUEUE_LIMIT then data is only sent if the amount of data available
 *    has reached 1/4 of the ETB (which for SYS_ETB is 8K). 
 *****************************************************************************/
void etb_add_queue(etb_queue_limit_t limit)
{
    LOGFUNC();
 
    bool is_wrapped;
    int etb_word_cnt = etb_data_available(&is_wrapped);

    if (etb_queue_cnt == max_queue_depth) {
        LOGMSG("%s:Max queue depth reached", __func__);
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    /* As we read the data with etb_process_data the etb rd pointer
     * will advance toward the etb write pointer so the next time
     * we check the number of words available should be much less
     */

    if (   ((limit == ETB_QUEUE_LIMIT) && (etb_word_cnt > etb_size/4))
        || ((limit == ETB_QUEUE_ALL) && (etb_word_cnt > 0))) {

        LOGMSG("%s:process %d ETB words", __func__, etb_word_cnt);
        etb_process_data(etb_word_cnt);
        return;
    }
   
    LOGMSG("%s:ETB available word count (%d) smaller than threshold (%d bytes) to read ETB", __func__);
    
}

/***************************************************************************** 
 *  etb_getbuf_from_queue()
 *
 *  This function retrieves etb data from the queue.
 * 
 *  - Get the etb buffer from the head of the queue.
 *  - Free the queue element, but not the buffer (calling routine should free the buffer).
 *  - Return NULL if no data on the queue.
 *****************************************************************************/
uint32_t * etb_getbuf_from_queue(uint32_t * ret_size_bytes)
{
    LOGFUNC(); 

    struct etb_queue_t * etb_queue_element;
    uint32_t * ret_buf_p;


    /* Get the etb buffer from the head of the queue. */
    if (etb_queue_head != NULL) {
        etb_queue_element = etb_queue_head;       
    } else {
        return NULL;
    }

    etb_queue_cnt--;

    ret_buf_p = (uint32_t *)etb_queue_element->buf_start;
    *ret_size_bytes = etb_queue_element->byte_cnt;

    etb_words_queued -= etb_queue_element->byte_cnt;

    /* Update the queue head */    
    etb_queue_head = etb_queue_head->next; 
    free(etb_queue_element);

    return ret_buf_p;
}

/***************************************************************************** 
 *  etb_savetofile_from_queue()
 *
 *  This function retrieves etb data from the queue and writes it to a file.
 * 
 *  - Get the etb buffer from the head of the queue.
 *  - Free the queue head.
 *  - Return NULL if no data on the queue.
 *****************************************************************************/
void etb_savetofile_queue()
{
    LOGFUNC(); 

    const size_t filename_max_size = 256;
    char bin_filename[filename_max_size];
    char dcm_filename[filename_max_size];

    struct etb_queue_t * etb_queue_element;

    util_gen_filename(bin_filename, filename_max_size, g_whoami, ".bin");
    util_gen_filename(dcm_filename, filename_max_size, g_whoami, ".dcm");

    LOGMSG("%s:Opening bin file %s", __func__, bin_filename);
    FILE * bin_fp = fopen(bin_filename, "wb");

    LOGMSG("%s:Opening dcm file %s", __func__, dcm_filename);
    FILE * dcm_fp = fopen(dcm_filename, "w");


    if ((bin_fp == NULL) || (dcm_fp == NULL)){
        if (bin_fp) fclose(bin_fp);
        if (dcm_fp) fclose(dcm_fp);
        LOGMSG("%s: Error opening bin or dcm file", __func__);
        return;
    }

    /* Get the etb buffer from the head of the queue (oldest data first). */
    if (etb_queue_head != NULL) {
        etb_queue_element = etb_queue_head;       
    } else {
        LOGMSG("%s:No data in queue", __func__);
        fclose(bin_fp);
        fclose(dcm_fp);
        return;
    }

    LOGMSG("%s:Write data from queue to bin file", __func__);

    while (etb_queue_element != NULL) {

        void * buf_p = (void *)etb_queue_element->buf_start;
        size_t element_cnt = etb_queue_element->byte_cnt/4;

        size_t fret = fwrite(buf_p, 4, element_cnt, bin_fp);

        if (fret != element_cnt) {
            fclose(bin_fp);
            fclose(dcm_fp);
            LOGMSG("%s:fret %d not equal to element_cnt %d - bin file write failure.", __func__, fret, element_cnt );
            char * msg = "Binary file write failed";
            err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
        }

        /* Update the queue head */    
        etb_queue_element = etb_queue_element->next; 
    }

    /* Write the dcm file */
    ulm_dcm_info_t dcm_info;

    ulm_get_dcminfo(&dcm_info);

    fprintf(dcm_fp, "STM_data_flip=%d\n", dcm_info.stm_data_flip);
    /* Never writing a wrapped buffer to the bin file */
    fprintf(dcm_fp, "STM_Buffer_Wrapped=%d\n", false);
    fprintf(dcm_fp, "HEAD_Present_0=%d\n", dcm_info.atb_head_present_0);
    fprintf(dcm_fp, "HEAD_Pointer_0=%d\n", dcm_info.atb_head_pointer_0);
    fprintf(dcm_fp, "HEAD_Present_1=%d\n", dcm_info.atb_head_present_1);
    fprintf(dcm_fp, "HEAD_Pointer_1=%d\n", dcm_info.atb_head_pointer_1);
    fprintf(dcm_fp, "STM_STP_Version=%d\n",dcm_info.stp_version);
    fprintf(dcm_fp, "TWP_Protocol=%d\n", dcm_info.using_twp);

    fclose(bin_fp);
    fclose(dcm_fp);
    return;
}

/***************************************************************************** 
 *  Private functions
 *  
 *****************************************************************************/

static ETBStatus etb_status_previous;

/***************************************************************************** 
 *  etb_data_available()
 *
 *  - If not TEST_MODE then get the number of words available from ETB_status()
 *    and determine if the ETB has wrapped.
 *  - If TEST_MODE then get a random number of words.
 * 
 *****************************************************************************/
static int etb_data_available(bool * wrapped)
{
    ETBStatus etb_status;

    LOGFUNC();

    if (global_state.recording == STATE_UNINITIALIZED) {
        LOGMSG("%s:Recording state is STATE_UNINITIALIZED so returning 0",
                __func__);
        return 0;
    }

    {
        eETB_Error  etb_ret;
        etb_ret = ETB_status(etb_handle, &etb_status);
        if (etb_ret != eETB_Success) {
            char * msg = "ETB status failure";

            LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
            err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
        }

        if ((g_log_enable) && 
           ((etb_status_previous.canRead != etb_status.canRead)
           || (etb_status_previous.isWrapped != etb_status.isWrapped)
           || (etb_status_previous.availableWords != etb_status.availableWords)
           || (etb_status_previous.ETB_TraceCaptureEn != etb_status.ETB_TraceCaptureEn)
           || (etb_status_previous.overflow != etb_status.overflow))) {

            LOGMSG("%s:etb status is:", __func__);
            LOGMSG("%s: canRead              %d", __func__, etb_status.canRead);
            LOGMSG("%s: isWrapped            %d", __func__, etb_status.isWrapped);
            LOGMSG("%s: availableWords       %d", __func__, etb_status.availableWords);
            LOGMSG("%s: ETB_TraceCaptureEn 0x%x", __func__, etb_status.ETB_TraceCaptureEn);
            LOGMSG("%s: overflow           0x%x", __func__, etb_status.overflow);

            etb_status_previous = etb_status;
        }
    }

    /* ETB wrapped status is only valid while the etb is enabled. Once the ETB 
     * is disabled and any data read the wrapped status is not valid from ETBLib.
     */
    if (etb_is_enabled) {
        *wrapped = (etb_status.isWrapped) ? true : false;
    }  else {
        *wrapped = etb_disabled_wrapped;
    }

    if ((global_state.etb_mode == ETB_MODE_DRAIN) && (etb_status.overflow)) {

        LOGMSG("%s:ETB_status call detected a ETB buffer overflow", __func__);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_OVERFLOW, NULL);
        
    }

    LOGMSG("%s:etb words available %d", __func__, etb_status.availableWords);
    return etb_status.availableWords;

}

/***************************************************************************** 
 *  etb_process_data()
 *
 *  This function Reads the ETB data and puts it into a buffer. The buffer is
 *  saved in a link list (etb_queue_t). 
 * 
 *  - Malloc a queue element.
 *  - Malloc a buffer to hold the ETB data.
 *  - If not TEST_MODE Read the ETB Data into the buffer.
 *  - If TEST_MODE then read the ETB data from the binary file.
 *  - Converted the ETB Data to network endianess.
 *  - Add the buffer to queue tail (oldest data at head, newest data at tail).
 *  - Since data is queued add the etb_socket to the list of 
 *    write sockets to test for ready.
 *****************************************************************************/
static void etb_process_data(int etb_word_cnt)
{
    LOGFUNC();    
    
    eETB_Error  etb_ret = eETB_Success;
    uint32_t read_size;
    struct etb_queue_t * etb_queue_element;

    /* Malloc a queue element. */
    etb_queue_element = malloc(sizeof(struct etb_queue_t));
    if  (etb_queue_element == NULL) {
        err_handler(ERR_TYPE_SYSTEM, ERR_MEM_ALLOC, NULL);
        /* Dummy return to keep code checker from declaring a defect,
         * err_handler() will exit for fatal errors.
         */
        return;
    }

    /* Malloc a buffer to hold the ETB data. */
    etb_queue_element->byte_cnt = etb_word_cnt * sizeof(uint32_t);
    if (etb_queue_element->byte_cnt == 0) {
        free(etb_queue_element);
        LOGMSG("%s:No ETB data available", __func__);
        return;
    }
    
    etb_queue_element->buf_start = malloc(etb_queue_element->byte_cnt);
    if  (etb_queue_element->buf_start == NULL) {
        free(etb_queue_element);
        err_handler(ERR_TYPE_SYSTEM, ERR_MEM_ALLOC, NULL);
        /* Dummy return to keep code checker from declaring a defect,
         * err_handler() will exit for fatal errors.
         */
        return;
    }
    etb_queue_element->buf_ptr = etb_queue_element->buf_start;

    /* Read the ETB Data into the buffer. */
   
    etb_ret = ETB_read(etb_handle, etb_queue_element->buf_start, etb_word_cnt, 0,
                       etb_word_cnt, &read_size);

    if(etb_ret != eETB_Success) {
        char * msg = "ETB_read failure";

        LOGMSG("%s:%s is %d", __func__, msg, etb_ret);
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ISSUE, msg);
    }

    /* Add the buffer to the queue. */     
    etb_queue_element->next = NULL;
    if (etb_queue_head == NULL) {
        etb_queue_head = etb_queue_element;
        etb_queue_tail = etb_queue_element;
    } else {
        etb_queue_tail->next = etb_queue_element;
        etb_queue_tail = etb_queue_element;
    }

    etb_queue_cnt++;
    etb_words_queued += etb_word_cnt;
    
}


