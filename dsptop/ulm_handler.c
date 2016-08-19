/*
 * ulm_handler.c
 *
 * dsptop implementation
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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "dsptop.h"
#include "etb_handler.h"
#include "error_handler.h"
#include "error.h"
#include "debug_log.h"
#include "stp_decode_handler.h"
#include "tiulm.h"
#include "io.h"
#include "ulm_handler.h"
#include "plot.h"

/******************************************************************************
 * Private Declarations
 *****************************************************************************/

/* These macros, when set to 1, include additional logging. 
 * Warning: setting these when dsps are generating a high ulm message rate
 * can cause dsptop to hang.
 */
#define MORE_LOGGING 0
#define LOG_TWP_STRIPPING 0

static dsp_type_t * config_dsp_p = NULL;
static size_t config_etb_size = 0;
static bool config_logging_mode = false;
static bool config_stop_full = false;
static int config_num_dsps = 0;
static double config_mva_window = 0;
uint32_t config_max_filesize = 0;
static bool config_process_until_exitmsg = false;
static bool sys_resources_configured = false;

static dsp_usage_info_t * config_dsp_usage_info_p = NULL;

static struct timeval delta_gap_tv;
static double gap;
static double recording_start;
static double recording_end;
static double sync_recording_time;

/* This is time applied to usage when recording is not stopped */
static double advance_time;  

/* Moving Average (mva) run and idle time sample buffers:
 * - Each entry in the mva bufs is the amount of run and idle time over a .1 seconds timespan
 * - The mva buffers are circular
 */
static int mva_sample_rate = 0;                 /* Number of samples per second */ 
static int mva_num_tv_samples = 0;              /* Total number of samples that can be put in the mva run and idle time bufs */
static struct timeval * mva_run_tv_p = NULL;    /* Pointer to the mva run time buf */
static struct timeval * mva_idle_tv_p = NULL;   /* Pointer to the mva idle time buf */
static int mva_index = 0;                       /* Points to the last valid sample in circular mva run and idle time bufs */
static bool mva_wrapped = false;                /* Indicates the mva bufs have wrapped */
static bool mva_first_sample = true;            /* Used to not add in gap or dead time for first trace buffer */ 


/* Flags that must be preserved between calls to ulm_decode_usage() */
static int const max_num_dsps = 8;
bool run_time_flag[] = {0,0,0,0,0,0,0,0};
bool idle_time_flag[] = {0,0,0,0,0,0,0,0};
bool exit_flag[] = {0,0,0,0,0,0,0,0};
#if defined(DRA7xx)
bool suspend_flipped[] = {false,false,false,false,false,false,false,false};
char *power_control_file[] = {
        "/sys/bus/platform/devices/40800000.dsp/power/control",
        "/sys/bus/platform/devices/41000000.dsp/power/control",
        NULL,NULL,NULL,NULL,NULL,NULL};
#endif

static double plot_runtime[] = {0,0,0,0,0,0,0,0};
static double plot_idletime[] = {0,0,0,0,0,0,0,0};
static double plot_timestamp[] = {0,0,0,0,0,0,0,0};

static double plot_resolution = .1;
static bool plot_usage_enable = false;

/********** timeval macros **********/

/* Provide timeval as a double
 * usage:
 * struct timeval tv 
 * double value = gettime(tv);
 */
#define gettime(tv) (gettimeofday(&tv, NULL),\
                    tv.tv_usec/1e6 + tv.tv_sec)

/* sum += value where sum and value are timeval */
#define sum_tv_to_tv(sum, value); sum.tv_sec += value.tv_sec;\
                    sum.tv_usec += value.tv_usec;\
                    if (sum.tv_usec >= 1000000) {\
                        sum.tv_usec -= 1000000;\
                        sum.tv_sec++;\
                    }

/* sum += sec.usec where sec and usec are doubles, sum is timeval. */
#define sum_double_to_tv(sum, sec, usec); sum.tv_usec += (long)usec;\
                    sum.tv_sec += (long)sec;\
                    if(sum.tv_usec >= 1000000) {\
                        sum.tv_usec -= 1000000;\
                        sum.tv_sec++;\
                    }

/* tv = 0 */
#define clear_tv(tv); tv.tv_sec = 0;\
                      tv.tv_usec = 0;

/******************************************************************************
 * Private Functions
 *****************************************************************************/
static inline void ulm_error_handler(ulm_error_t error);
static void mva_total_usage();

static void ulm_plot_usage(int dsp_index, struct timeval * delta_tv_p);
static void ulm_plot_clear();

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/***************************************************************************** 
 *  ulm_init
 *
 *  Initialize the ulm handler for the mode (mva, total or logging)
 *
 *  Note: This function may be called at any time to change mode or resize the
 *        number of sample points for mva mode.
 *
 *****************************************************************************/
void ulm_init(options_t * options_p, dsp_type_t * dsp_p, dsp_usage_info_t * dsp_usage_info_p)
{
    LOGFUNC();

    if (dsp_p->num_dsps > max_num_dsps) {
        err_handler(ERR_TYPE_LOCAL,ERR_DEBUG, NULL);
    }

    /* Check if option changed from Moving Average Mode */
    if ((config_mva_window != 0) && ((options_p->mva_window == 0) || options_p->logging_enable)) {
        /* Free old mva bufs */
        if (mva_run_tv_p != NULL) { 
            free(mva_run_tv_p);
            mva_run_tv_p = NULL;
            LOGMSG("%s: Free mva run buf", __func__);
        }

        if (mva_idle_tv_p != NULL) {
            free(mva_idle_tv_p);
            mva_idle_tv_p = NULL;
            LOGMSG("%s: Free mva idle buf", __func__);
        }

        LOGMSG("%s: Switch from MVA to either TOTAL or LOGGING", __func__);               
    }

    /* Check if not Logging and not Total Mode - must be Moving Average Mode*/
    if (!options_p->logging_enable && (options_p->mva_window != 0)) {

        int new_sample_rate = (options_p->mva_window > 60) ? 1 : 10;
        int new_num_tv_samples = (int)((double)dsp_p->num_dsps * (options_p->mva_window * (double)new_sample_rate));

        LOGMSG("%s: Init to MVA mode - new sample resolution %d, new num samples %d", 
              __func__, new_sample_rate, new_num_tv_samples);

        if (mva_run_tv_p != NULL) { 
            free(mva_run_tv_p);
            mva_run_tv_p = NULL;
            LOGMSG("%s: Free mva run buf", __func__);
        }

        if (mva_idle_tv_p != NULL) {
            free(mva_idle_tv_p);
            mva_idle_tv_p = NULL;
            LOGMSG("%s: Free mva idle buf", __func__);
        } 

        mva_run_tv_p = calloc(new_num_tv_samples, sizeof(struct timeval));
        mva_idle_tv_p = calloc(new_num_tv_samples, sizeof(struct timeval));

        if ((mva_run_tv_p == NULL) || (mva_idle_tv_p == NULL)) {
            err_handler(ERR_TYPE_SYSTEM,ERR_MEM_ALLOC, NULL);
            /* Dummy return to keep code checker from declaring
             * an issue, err_handler will call exit.
             */
            return;
        } 

        /* Fresh start */
        mva_num_tv_samples = new_num_tv_samples;
        mva_sample_rate = new_sample_rate;
        mva_index = 0;                       
        mva_wrapped = false;
        mva_first_sample = true;
              
    }

    config_dsp_p = dsp_p;    
    config_num_dsps = dsp_p->num_dsps;
    config_logging_mode = options_p->logging_enable;
    config_stop_full = options_p->stop_full;
    config_mva_window = (config_logging_mode) ? 0 : options_p->mva_window;
    config_dsp_usage_info_p = dsp_usage_info_p;
    config_max_filesize = options_p->max_filesize;
    config_process_until_exitmsg = options_p->process_until_exitmsg;

    if (sys_resources_configured) {
        ulm_term();
    }

    /* Initialize accumulated gap time - used for calculating accuracy */
    gap = 0;

    /* Initialize recording time */
    recording_start = 0;
    recording_end = 0;
    sync_recording_time = 0;

    /* Initialize advance time - Used to keep track of how much time has been advance while recording data */    
    advance_time = 0;

    /* Initialize delta gap time - Adding to initial elapsed time to extrapolate run_time_flag and idle_time_flag between ETB buffers */
    clear_tv(delta_gap_tv);

    for (int i = 0; i < config_num_dsps; i++) {
        run_time_flag[i] = false;
        idle_time_flag[i] = false;
        exit_flag[i] = false;
    }

    plot_resolution = options_p->plot_resolution;
    plot_usage_enable = ((options_p->out_filename_p != NULL) && (options_p->logging_enable == false)) ? true : false;
    ulm_plot_clear();

#if defined(DRA7xx)
    /* DRA7xx might have power control, set DSP to "on" if suspended */
    FILE *fp = NULL;
    char status[8];
    for (int i = 0; i < config_num_dsps; i++) {
        if (power_control_file[i] == NULL)  continue;
        if ((fp = fopen(power_control_file[i], "r+")) == NULL) continue;
        if (fread(status, 1, 2, fp) < 2)  continue;
        if (strncmp(status, "on", 2) == 0)  continue;
        fseek(fp, 0L, SEEK_SET);
        fwrite("on", 1, 2, fp);
        fclose(fp);
        suspend_flipped[i] = true;
        LOGMSG("DSP%d: power control flipped to on", i+1);
    }
#endif
}

/***************************************************************************** 
 *  ulm_term
 *
 *  Release resources used by the ulm handler functions.
 *
 *****************************************************************************/
void ulm_close()
{
    /* etb_term() and ulm_term() will unmap etb and stm xport space */
    if (sys_resources_configured) {
        etb_term();
        ulm_term();
        sys_resources_configured = false;

#if defined(DRA7xx)
        /* DRA7xx might have power control, set DSP to "auto" if flipped */
        FILE *fp = NULL;
        for (int i = 0; i < config_num_dsps; i++) {
            if (   suspend_flipped[i] == false
                || power_control_file[i] == NULL)  continue;
            if ((fp = fopen(power_control_file[i], "w")) == NULL) continue;
            fwrite("auto", 1, 4, fp);
            fclose(fp);
            suspend_flipped[i] = false;
            LOGMSG("DSP%d: power control flipped back to auto", i+1);
        }
#endif
    }

}
/***************************************************************************** 
 *  ulm_config_etb_recording
 *
 *  Configure STM and ETB for recording ULM messages
 *
 *  Note: STM and ETB are configured. STM and ETB are enabled for recording.
 *
 *****************************************************************************/
void ulm_config_etb_recording()
{
    LOGFUNC();

    /* Start ulm message acquisition for the mode*/
    if (config_logging_mode) {
        /* Setup for logging mode */
        etb_bufmode_t etb_mode = (config_stop_full) ? ETB_FIXED : ETB_CIRCULAR;
        config_etb_size = etb_config(etb_mode, SYS_TIETB);

    } else {
        /* Setup for either Moving Average or Total mode */
        config_etb_size = etb_config(ETB_CIRCULAR, SYS_TIETB);
    }
 
    etb_enable();

    struct timeval recording_start_tv;
    recording_start = gettime(recording_start_tv);

    /* If test mode map all the STM Xport space. 
     * The call to ulm_config() only maps the upper half of Xport
     * space for dsptop.
     */
    if (g_mva_test_mode != TEST_MODE_DISABLED) {
        ulm_test_map();
        LOGMSG("%s: ULM test mode mapping", __func__);
    }

    ulm_error_t ulm_ret = ulm_config();
    if (ulm_ret != ULM_SUCCESS) {
        ulm_error_handler(ulm_ret);
    }

    sys_resources_configured = true;
}

/***************************************************************************** 
 *  ulm_stop_etb_recording
 *
 *  Stop recording: Flush the STM and disable it from generating additional
 *  messages. Disable the ETB and get the ETB status.
 *
 *****************************************************************************/
int ulm_stop_etb_recording()
{

    int etb_avaiable_words;
    bool is_enabled;
    bool is_wrapped;

    LOGFUNC();

    /* Note: When the ETB is flushed from ETB_disable the STM FIFO and STM ATB bus
     * Interface are both flushesd to the ETB. Thus no STM flush (ulm_flush()) is necessary.
     */
    ulm_put_sync();
    etb_disable();

    ulm_error_t ulm_ret = ulm_disable();
    if (ulm_ret != ULM_SUCCESS) {
        ulm_error_handler(ulm_ret);
    }

    etb_avaiable_words =  etb_status(&is_enabled, &is_wrapped);

    return etb_avaiable_words;
}

/***************************************************************************** 
 *  ulm_restart_etb_recording
 *
 *  Stop recording: Flush the STM and disable it from generating additional
 *  messages. Disable the ETB and get the ETB status.
 *
 *****************************************************************************/
void ulm_restart_etb_recording()
{

    bool is_enabled;
    bool is_wrapped;

    LOGFUNC();

    etb_enable();

    ulm_error_t ulm_ret = ulm_enable();
    if (ulm_ret != ULM_SUCCESS) {
        ulm_error_handler(ulm_ret);
    }

    etb_status(&is_enabled, &is_wrapped);

}

/***************************************************************************** 
 *  ulm_decode_to_logfile
 *
 *  Decode ULM data to a log file. Return true if log file written,
 *  false if no data found after twp stripping.
 *
 *****************************************************************************/
bool ulm_decode_to_logfile(char * out_filename_p, dsp_type_t * dsp_info_p)
{
    LOGFUNC();

    /* Allocate a buffer for the twp stripped etb data */
    uint8_t etb_twpstrip_p[config_etb_size * 2];
    /* Storage for the actual twp stripped size */
    size_t etb_twpstrip_size = 0;  

    /* Queue the etb contents */
    etb_add_queue(ETB_QUEUE_ALL);

    /* Save bin to dsptop.bin and dcm data to dsptop.dcm */
    if (g_log_enable) {
        etb_savetofile_queue();
    }

    uint32_t etb_size_bytes = 0;
    uint32_t * etb_buf_p = etb_getbuf_from_queue(&etb_size_bytes);
    LOGMSG("%s: etb byte count is: %d", __func__, etb_size_bytes); 

    /**************  Decode *****************/
    ulm_dcm_info_t dcm_info;
    ulm_get_dcminfo(&dcm_info);

    stp_ulm_t * ulm_msg_p;
    int ulm_msg_cnt;

    stp_decode_open(&dcm_info, dsp_info_p);

    stp_decode_strip_twp(etb_buf_p, (size_t)etb_size_bytes, etb_twpstrip_p, &etb_twpstrip_size);

    LOGMSG("%s:etb buffer w/twp stripped size in bytes is %d", __func__, etb_twpstrip_size);

    if (etb_twpstrip_size == 0) {
        free(etb_buf_p);
        stp_decode_close();
        return false;  
    }

#if LOG_TWP_STRIPPING
    uint32_t * etb_twpstrip_wp = (uint32_t *)etb_twpstrip_p;
    uint32_t offset = 0;
    for (int i = 0; i < (etb_twpstrip_size/8); i++) {
        LOGMSG2("0x%08.8x 0x%08.8x 0x%08.8x", offset, *etb_twpstrip_wp, *(etb_twpstrip_wp+1));
        offset += 8;
        etb_twpstrip_wp += 2;
    }
    if ((etb_twpstrip_size % 8) == 4) {
        LOGMSG2("0x%08.8x 0x%08.8x", offset, *etb_twpstrip_wp);
    }    
#endif

    uint32_t etb_twpstrip_startbyte_pos = 0; 
    uint32_t etb_twpstrip_nextbyte_pos = 0; 
    bool etb_twpstrip_allbytes_processed = false;

    /* This will decode all etb data and provide a ulm_msg_p and ulm_msg_cnt */
    stp_decode_etb2ulm(etb_twpstrip_p, etb_twpstrip_size, etb_twpstrip_startbyte_pos, &etb_twpstrip_nextbyte_pos,
                       &etb_twpstrip_allbytes_processed, &ulm_msg_p, &ulm_msg_cnt);


    if (ulm_msg_cnt > 0) {

        bool append = false;
        struct timeval offset_tv = {0,0};

        stp_decode_log2file(ulm_msg_p, ulm_msg_cnt, out_filename_p, config_max_filesize, dsp_info_p, append, &offset_tv);

    }

    if (ulm_msg_p != NULL) {
        free(ulm_msg_p);
    }
    free(etb_buf_p);
    stp_decode_close();

    if (ulm_msg_cnt == 0) {
        LOGMSG("%s: No ulm packets found", __func__);
        return false;
    }

    LOGMSG("%s: ulm_msg_cnt is %d, etb_twpstrip_nextbyte_pos is %d, etb_twpstrip_allbytes_processed is %d",__func__, 
            ulm_msg_cnt, etb_twpstrip_nextbyte_pos, etb_twpstrip_allbytes_processed);

    if (!etb_twpstrip_allbytes_processed) {
        char * msg = "Not all etb data processed";
        LOGMSG("%s:%s", __func__, msg);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);            
    }

    return true;
}

/***************************************************************************** 
 *  ulm_decode_usage
 *
 *  Decode ULM data and fill in usage structure
 *
 * Note (Moving Average Mode): This mode accumulates run/idle mva_num_tv_samples
 * in a circular buffer where each individual run/idle time elements are simply
 * added to get the total run/idle time for the usage period (where the combination
 * of the run and idle time can not exceed the usage period). There is a bit of
 * inaccuracy built-in because the current run/idle time is cleared before it is
 * updated and the last ulm samples will normally not have enough data to cover
 * the resolution of the current element.
 *
 *****************************************************************************/
void ulm_decode_usage()
{
    LOGFUNC();

    int etb_percent_full;
    bool is_etb_enabled;
    bool is_etb_wrapped;
    bool is_logging;
    bool stop_trigger;

    double gap_start, gap_end;
    double recording_time;

    /* Allocate a buffer for the twp stripped etb data */
    uint8_t etb_twpstrip_p[config_etb_size * 2];

    /* Pause sampling */
    ulm_stop_etb_recording();

    /* Get start time for gap calculation */
    struct timeval gap_start_tv;
    gap_start = gettime(gap_start_tv);
    recording_end = gap_start;

    /* Calculate recording time */
    recording_time = recording_end - recording_start;

    /* Get etb state infomation */
    ulm_get_etb_state(&etb_percent_full, &is_etb_enabled, &is_etb_wrapped, &is_logging, &stop_trigger);
    if (is_etb_enabled == true) {
        err_handler(ERR_TYPE_LOCAL, ERR_ETB_ENABLED_STATE, NULL);    
    } 

    /* Queue the etb contents */
    etb_add_queue(ETB_QUEUE_ALL);

    uint32_t etb_size_bytes = 0;
    uint32_t * etb_buf_p = etb_getbuf_from_queue(&etb_size_bytes);
    LOGMSG("%s: etb byte count is: %d", __func__, etb_size_bytes); 

    /* Restart sampling */
    ulm_restart_etb_recording();

    /* Get end time and calculate gap */
    struct timeval gap_end_tv;
    gap_end = gettime(gap_end_tv);
    recording_start = gap_end;

    /* gap is provided by ulm_get_gaptime() used in io.c to calculate accuracy */
    gap += gap_end - gap_start;

    LOGMSG("%s:accumulated gap time is %f", __func__, gap); 

    /* Decode ETB samples */
    size_t etb_twpstrip_size = 0; 

    /**************  Decode *****************/
    ulm_dcm_info_t dcm_info;
    ulm_get_dcminfo(&dcm_info);

    stp_ulm_t * ulm_msg_p;
    int ulm_msg_cnt;

    stp_decode_open(&dcm_info, config_dsp_p);

    stp_decode_strip_twp(etb_buf_p, (size_t)etb_size_bytes, etb_twpstrip_p, &etb_twpstrip_size);

    if (etb_twpstrip_size == 0) {
        free(etb_buf_p);
        stp_decode_close();
        return;  
    }

#if LOG_TWP_STRIPPING
    uint32_t * etb_twpstrip_wp = (uint32_t *)etb_twpstrip_p;
    uint32_t offset = 0;
    for (int i = 0; i < (etb_twpstrip_size/8); i++) {
        LOGMSG2("0x%08.8x 0x%08.8x 0x%08.8x", offset, *etb_twpstrip_wp, *(etb_twpstrip_wp+1));
        offset += 8;
        etb_twpstrip_wp += 2;
    }
    if ((etb_twpstrip_size % 8) == 4) {
        LOGMSG2("0x%08.8x 0x%08.8x", offset, *etb_twpstrip_wp);
    }  
#endif

    uint32_t etb_twpstrip_startbyte_pos = 0; 
    uint32_t etb_twpstrip_nextbyte_pos = 0; 
    bool etb_twpstrip_allbytes_processed = false;

    bool first_valid_sample[config_num_dsps];
    struct timeval previous_tv[config_num_dsps];
    struct timeval current_tv;
    struct timeval delta_tv;

    /* Initialize for total mode - will change later if mva */
    struct timeval * run_tv_p = config_dsp_usage_info_p->run_tv_p;
    struct timeval * idle_tv_p = config_dsp_usage_info_p->idle_tv_p;
    struct timeval elapsed_tv = {
        .tv_sec = 0,
        .tv_usec = 0
    };

    double mva_threshold = (mva_sample_rate == 10) ? 0.1 : 1.0;
    LOGMSG("%s: mva threshold is %f", __func__, mva_threshold);
  
    for (int i = 0; i < config_num_dsps; i++) {
        previous_tv[i].tv_usec = 0;
        previous_tv[i].tv_sec = 0;
    }

    /* If !mva_first_sample, update with gap time else leave elapse_tv 0 */
    if (!mva_first_sample) {
        elapsed_tv = delta_gap_tv;
        LOGMSG("%s: Set initial elapsed_tv tp delta gap time %d.%06d", __func__, delta_gap_tv.tv_sec, delta_gap_tv.tv_usec);
    }

    /* This will decode all etb data and provide a ulm_msg_p and ulm_msg_cnt */
    stp_decode_etb2ulm(etb_twpstrip_p, etb_twpstrip_size, etb_twpstrip_startbyte_pos, &etb_twpstrip_nextbyte_pos,
                       &etb_twpstrip_allbytes_processed, &ulm_msg_p, &ulm_msg_cnt);

    g_total_ulm_msgcnt += ulm_msg_cnt;

    /* Check for no ulm messages in the data or not all etb data was processed */
    if ((ulm_msg_cnt == 0)  || !etb_twpstrip_allbytes_processed) {
        if (ulm_msg_p != NULL) {
            free(ulm_msg_p);
        }
        free(etb_buf_p);
        stp_decode_close();

        if (ulm_msg_cnt == 0) {

            /* Must be nothing but sync messgaes so keep track of recording_time */
            sync_recording_time += recording_time;

            LOGMSG("%s: No ulm packets found - new sync recording time is %f", __func__, sync_recording_time);
            return;
        }

        /* etb_twpstrip_allbytes_processed must be false */
        char * msg = "Not all etb data processed";
        LOGMSG("%s:%s", __func__, msg);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);            
    }

    /* If there was previous sync_recording_time then add it in to current recording time */
    if (sync_recording_time != 0) {
        recording_time += sync_recording_time;
        sync_recording_time = 0;

        LOGMSG("%s: Added sync time %f to recoding time - new recording time is %f", __func__, 
               sync_recording_time, recording_time);
    }

    /* If test mode then adjust ARM master id to DSP id */
    if (g_mva_test_mode != TEST_MODE_DISABLED) {
        LOGMSG("%s: mva test mode - adjust arm master ids a DSP id", __func__);
        int new_master_id = 0;
        for ( int i = 0; i < ulm_msg_cnt; i++) {
#ifdef C66AK2Hxx
            if (ulm_msg_p[i].master_id >= 8) {
#elif DRA7xx
            if ((ulm_msg_p[i].master_id == 0) || (ulm_msg_p[i].master_id == 1)) {
#else
            #error "Invalid DEVICE type"
#endif
                ulm_msg_p[i].master_id = new_master_id++;
            }
            if (new_master_id == config_num_dsps) {
                new_master_id = 0;
            }
        }
    } 

    /* Initialize run and idle time sample pointers for the mode */
    if (config_mva_window == 0) {
        /* Total mode so accumulate run and idle time over all samples */
        LOGMSG("%s: Total mode", __func__);
        run_tv_p = config_dsp_usage_info_p->run_tv_p;
        idle_tv_p = config_dsp_usage_info_p->idle_tv_p;
    } else {
        /* MVA mode so if not the first time, increment the run and idle time sample mva_index
         * to the next, avoid writing over data from the previous session.
         */
        run_tv_p = mva_run_tv_p + (mva_index * config_num_dsps);
        idle_tv_p = mva_idle_tv_p + (mva_index * config_num_dsps);

        LOGMSG("%s: MVA mode - Starting index is %d", __func__, mva_index * config_num_dsps);
    }

    for ( int i = 0; i  < config_num_dsps; i++) {
        first_valid_sample[i] = true;         
    }

    bool first_valid_sample_mva_advance = true;

    /* Subtract time captured for samples in the etb from the recording time and add that in
     * to elapsed time. This is what accounts for the fact that every ETB buffer restarts at
     * timestamp 0.
     */
    double first_sample_time = ulm_msg_p[0].tv.tv_usec/1e6 + ulm_msg_p[0].tv.tv_sec;
    double last_sample_time = ulm_msg_p[ulm_msg_cnt-1].tv.tv_usec/1e6 + ulm_msg_p[ulm_msg_cnt-1].tv.tv_sec;

    /* This is the time between the start of recording and the first sample */
    double dead_time = recording_time - (last_sample_time - first_sample_time);

    dead_time -= advance_time;

    /* dead_time may be less than 0 (on the order of micro-seconds) just because of STM timestamp clock scaling */
    if (dead_time < 0) {
        dead_time = 0;
    } 

    LOGMSG("%s: advance time is %f, final dead_time (minus advance_time) is %f, recording time is %f, first sample time is %f, last sample time is %f",
            __func__, advance_time, dead_time, recording_time, first_sample_time, last_sample_time);

    advance_time = 0;

    /* Only add dead time to elapsed time if not first buffer */
    if (!mva_first_sample) {
        double dead_time_sec;
        double dead_time_usec = modf(dead_time, &dead_time_sec) * 1e6;

        sum_double_to_tv(elapsed_tv, dead_time_sec, dead_time_usec);

        LOGMSG("%s: Elapsed time (gap + dead time) is %d.%06d", __func__, elapsed_tv.tv_sec, elapsed_tv.tv_usec); 
    }

    /* loop through each sample */
    for (int i = 0; i < ulm_msg_cnt; i++) {

        //mid must be in the range of 0 to config_num_dsps-1.
        uint8_t mid;
        for (mid = 0; mid < config_dsp_p->num_masters; mid++) {
            if (ulm_msg_p[i].master_id == config_dsp_p->master_id[mid]) {
                break;
            }
        } 

        /* If the current sample is not from a DSP then skip time advance */
        if (mid <= (config_num_dsps-1)) {

            /* Has this sample been set as previous */
            bool is_previous_set = false;

            /* If MVA and current packet valid */
            if ((config_mva_window != 0) && (ulm_msg_p[i].ulm_usage != -1)) {

                /* If the exit flag true (must be in Process mode AND there is a
                 * idle or run message - then time to start over.
                 */
                if ((exit_flag[mid] == true) && ((ulm_msg_p[i].ulm_usage == ULM_STATE_IDLE) || (ulm_msg_p[i].ulm_usage == ULM_STATE_RUN))) {

                    run_tv_p = mva_run_tv_p;
                    idle_tv_p = mva_idle_tv_p;

                    for (int j = 0; j < (mva_num_tv_samples/config_num_dsps); j++) {
                        clear_tv(idle_tv_p[mid]);
                        clear_tv(run_tv_p[mid]);

                        run_tv_p += config_num_dsps;
                        idle_tv_p += config_num_dsps;
                    }

                    /* will not hurt anythng to clear this for every occurance since it's already set
                     * to zero if not the first valid sample - see below.
                     */
                    clear_tv(elapsed_tv);

                    run_tv_p = mva_run_tv_p;
                    idle_tv_p = mva_idle_tv_p;
                    mva_index = 0;

                    exit_flag[mid] = false;

                    if (plot_usage_enable) {
                        ulm_plot_clear();
                        plot_start();
                    }

                    LOGMSG2("%s: MVA exit - clear run/idle times", __func__);
                }            

                /* Determine delta from the previous sample and figure 
                 * out the elapsed time for this sample
                 */

                LOGMSG2("%s: MVA mode processing sample %d mid %d ", __func__, i, mid);

                struct timeval mva_previous;

                if (first_valid_sample_mva_advance) {
                    clear_tv(mva_previous);
                    first_valid_sample_mva_advance = false;
                } else {
                    mva_previous = previous_tv[mid];
                    clear_tv(elapsed_tv);
                }

                LOGMSG2("%s:Elapse time %d.%06d", __func__, elapsed_tv.tv_sec, elapsed_tv.tv_usec);
                LOGMSG2("%s:Previous time %d.%06d", __func__, mva_previous.tv_sec, mva_previous.tv_usec);

                /* Calculate the elapsed time between samples.
                 * Note: for first time through elapsed_tv already includes gap time 
                 * plus the difference between recording time and sample time thus +=.
                 */
                if (mva_previous.tv_usec > ulm_msg_p[i].tv.tv_usec) {
                    current_tv.tv_usec = ulm_msg_p[i].tv.tv_usec + 1000000;
                    current_tv.tv_sec = ulm_msg_p[i].tv.tv_sec - 1;
                    
                } else {
                    current_tv = ulm_msg_p[i].tv;
                }
                elapsed_tv.tv_usec += current_tv.tv_usec - mva_previous.tv_usec;
                elapsed_tv.tv_sec += current_tv.tv_sec - mva_previous.tv_sec;
                if (elapsed_tv.tv_usec >= 1000000) {
                    elapsed_tv.tv_usec -= 1000000;
                    elapsed_tv.tv_sec += 1;
                }

                LOGMSG2("%s:Current time %d.%06d", __func__, current_tv.tv_sec, current_tv.tv_usec);
                LOGMSG2("%s:Final Elapsed time %d.%06d", __func__, elapsed_tv.tv_sec, elapsed_tv.tv_usec);

                /* If advancing time for this dsp check if one of the special cases applies */
                if (run_time_flag[mid] || idle_time_flag[mid]) {

                    /* If the first sample, the total run_tv + idle_tv + the next sample time > mva_threshold or the last sample,
                     * then apply the difference to each DSP (normalize all DSPs to this sample).
                     * Normalizing all dsp time to the current sample allows the mva_index to be incremented
                     * for all DSPs together.
                     */

                    double runtime = run_tv_p[mid].tv_usec/1e6 + run_tv_p[mid].tv_sec;
                    double idletime = idle_tv_p[mid].tv_usec/1e6 + idle_tv_p[mid].tv_sec;
                    double elapsedtime = elapsed_tv.tv_usec/1e6 + elapsed_tv.tv_sec;
                    double newtime = runtime + idletime + elapsedtime;

                    if ((newtime > mva_threshold) || (i == (ulm_msg_cnt-1) || (i == 0))){

                        LOGMSG2("%s:runtime %f, idletime %f, elaspedtime %f, newtime %f", __func__, 
                               runtime, idletime, elapsedtime, newtime);

                        bool recoding_stopped = true;

                        double current = ulm_msg_p[i].tv.tv_usec/1e6 + ulm_msg_p[i].tv.tv_sec;
                        double previous = mva_previous.tv_usec/1e6 + mva_previous.tv_sec;
                        /* elapsed time includes gap and advance time */
                        double elapsed = elapsedtime - (current - previous);
                        if (elapsed < 0) {
                            elapsed = 0;
                        }
                        ulm_advance_usage(elapsed, current, previous_tv, recoding_stopped);

                        /* If this is the first sample then elapsedtime is simply deadtime + gaptime
                         * and if we include this time in previous for each DSP, the next sample will not get added
                         * in to run/idle time because the delta time will be negative.
                         */
                        if (i == 0) {
                            for (int j = 0; j  < config_num_dsps; j++) {
                                if (j != mid ) {
                                    previous_tv[j] = ulm_msg_p[i].tv;
                                }
                            }

                        }

                        /* Update the run and idle time pointers in case ulm_advance_usage() updated the mva_index */
                        run_tv_p = mva_run_tv_p + (mva_index * config_num_dsps);
                        idle_tv_p = mva_idle_tv_p + (mva_index * config_num_dsps);

                        clear_tv(elapsed_tv);

                        is_previous_set = true;

                    } /* End of - If MVA and current packet valid */

                }

                /* End if MVA */
            } else {

                /* If the exit flag true (must be in Process mode AND there is a
                 * idle or run message - then time to start over.
                 */
                if ((exit_flag[mid] == true) && ((ulm_msg_p[i].ulm_usage == ULM_STATE_IDLE) || (ulm_msg_p[i].ulm_usage == ULM_STATE_RUN))) {

                    clear_tv(idle_tv_p[mid]);
                    clear_tv(run_tv_p[mid]);
                    clear_tv(elapsed_tv);

                    exit_flag[mid] = false;

                    if (plot_usage_enable) {
                        ulm_plot_clear();
                        plot_start();
                    }

                    LOGMSG2("%s:Total Mode exit clear run/idle times", __func__);
                }            

                /* End of Total Mode */
            }

            /* Accumulate run and idle time per dsp core */

            /* Determine delta time if previous ulm message was ULM_STATE_RUN or ULM_STATE_IDLE 
             * and next message is either ULM_STATE_RUN, ULM_STATE_IDLE, or ULM_STATE_EXIT
             */

            if (  ((idle_time_flag[mid] == true) || (run_time_flag[mid] == true))
               && ((ulm_msg_p[i].ulm_usage == ULM_STATE_IDLE) || (ulm_msg_p[i].ulm_usage == ULM_STATE_RUN) 
                  || (ulm_msg_p[i].ulm_usage == ULM_STATE_EXIT))) {

                if (first_valid_sample[mid] == true) {
                    /* previous_tv already set to 0 */
                    current_tv = elapsed_tv;
                    first_valid_sample[mid] = false;
                } else {
                    current_tv = ulm_msg_p[i].tv;

                    if (previous_tv[mid].tv_usec > current_tv.tv_usec) {
                        current_tv.tv_usec = current_tv.tv_usec + 1000000;
                        current_tv.tv_sec = current_tv.tv_sec - 1;                        
                    }
                }

                delta_tv.tv_usec = current_tv.tv_usec - previous_tv[mid].tv_usec;
                delta_tv.tv_sec = current_tv.tv_sec - previous_tv[mid].tv_sec;

                /* If the sample that advanced the mva_index has the same timestamp as the next sample,
                 * which can happen because the resolution on timestamps is only usec, then the previous_tv
                 * value already has the time for this sample applied (thus previous_tv may be slightly larger than 
                 * the current_tv because of rounding) causing a negative delta_tv value that will simply be ignored.
                 */
                bool add_delta_flag = (delta_tv.tv_usec < 0 || delta_tv.tv_sec < 0) ? false: true;

                /* NOTE - negative values can show up in the log file, see previous note !!! */
                LOGMSG2("%s:Sample %d DSP_%d", __func__, i, mid)
                LOGMSG2("%s:  Original time      %d.%06d", __func__, ulm_msg_p[i].tv.tv_sec, ulm_msg_p[i].tv.tv_usec);
                LOGMSG2("%s:  Previous time      %d.%06d", __func__, previous_tv[mid].tv_sec, previous_tv[mid].tv_usec);
                LOGMSG2("%s:  Current time       %d.%06d", __func__, current_tv.tv_sec, current_tv.tv_usec);
                LOGMSG2("%s:  Delta time         %d.%06d", __func__, delta_tv.tv_sec, delta_tv.tv_usec);
                
                /* Add delta to run time */
                if (add_delta_flag && (run_time_flag[mid] == true)) {

                    /* run_tv_p[mid] +=  delta_tv */
                    sum_tv_to_tv(run_tv_p[mid], delta_tv);
         
                    LOGMSG2("%s:  New run time       %d.%06d", __func__, run_tv_p[mid].tv_sec, run_tv_p[mid].tv_usec);
                }

                /* Add delta to idle time */
                if (add_delta_flag && (idle_time_flag[mid] == true)) {

                    /* idle_tv_p[mid] +=  delta_tv */
                    sum_tv_to_tv(idle_tv_p[mid], delta_tv);                    

                    LOGMSG2("%s:  New idle time      %d.%06d", __func__, idle_tv_p[mid].tv_sec, idle_tv_p[mid].tv_usec);
                }


                /* If previous is smaller than the current sample then update  previous */
                if ((add_delta_flag) && (!is_previous_set)) {

                    previous_tv[mid] = ulm_msg_p[i].tv;

                    LOGMSG2("%s:  NEW previous time  %d.%06d", __func__, previous_tv[mid].tv_sec, previous_tv[mid].tv_usec);

                    is_previous_set = true;
                }

                /* Plot usage */
                if (plot_usage_enable && add_delta_flag && (run_time_flag[mid] || idle_time_flag[mid])) {
                    ulm_plot_usage(mid, &delta_tv);
                }    

            }

            /* Current message is ULM_STATE_IDLE */
            if (ulm_msg_p[i].ulm_usage == ULM_STATE_IDLE) {
                run_time_flag[mid] = false;
                idle_time_flag[mid] = true;
            }

            /* Current message is ULM_STATE_RUN */
            if (ulm_msg_p[i].ulm_usage == ULM_STATE_RUN) {
                run_time_flag[mid] = true;
                idle_time_flag[mid] = false;
            }

            /* Current message is ULM_STATE_EXIT and not Process mode - treat as idle */
            if ((ulm_msg_p[i].ulm_usage == ULM_STATE_EXIT) && (!config_process_until_exitmsg)) {
                run_time_flag[mid] = false;
                idle_time_flag[mid] = true;
            }

            /* Current message is ULM_STATE_EXIT and Process mode - terminate run/idle time accumulation */
            if ((ulm_msg_p[i].ulm_usage == ULM_STATE_EXIT) && (config_process_until_exitmsg)) {
                run_time_flag[mid] = false;
                idle_time_flag[mid] = false;
                exit_flag[mid] = true;

                LOGMSG2("%s:Sample %d Mid %d Detected exit in Process Mode", __func__, i, mid);
            }

            /* If first sample for this dsp set previous_tv for next pass */
            if (  ((ulm_msg_p[i].ulm_usage == ULM_STATE_IDLE) || (ulm_msg_p[i].ulm_usage == ULM_STATE_RUN)) 
               && !is_previous_set) {

                previous_tv[mid] = ulm_msg_p[i].tv;
                is_previous_set = true;

                LOGMSG2("%s:Sample %d Mid %d No previous_tv set - NEW previous_tv is %d.%06d", __func__, i, mid,
                       previous_tv[mid].tv_sec, previous_tv[mid].tv_usec);

            }

        }

        /* The following messages are valid from any core */

        /* Current message is ULM_MEM_EX_CODE_AND_DATA */
        if (  (ulm_msg_p[i].ulm_pk_type == ULM_MEM_EX_CODE_AND_DATA) 
           && (ulm_msg_p[i].value_1_valid == true)) {

            stp_decode_memvalue(ulm_msg_p[i].value_1,
                                &config_dsp_usage_info_p->extern_cd.num_1kb_blocks,
                                &config_dsp_usage_info_p->extern_cd.percent_free);

            if (config_dsp_usage_info_p->extern_cd.percent_free < config_dsp_usage_info_p->extern_cd.min_percent_free) {
                config_dsp_usage_info_p->extern_cd.min_percent_free = config_dsp_usage_info_p->extern_cd.percent_free;
            }

             LOGMSG2("%s: Sample %d Mid %d Value is %x External Code/Data %d 1KB Blocks %3.1f%% Free", __func__, i, mid, 
                    ulm_msg_p[i].value_1,
                    config_dsp_usage_info_p->extern_cd.num_1kb_blocks,
                    config_dsp_usage_info_p->extern_cd.percent_free);  
        }

        /* Current message is ULM_MEM_EX_DATA_ONLY */
        if (  (ulm_msg_p[i].ulm_pk_type == ULM_MEM_EX_DATA_ONLY) 
           && (ulm_msg_p[i].value_1_valid == true)) {

            stp_decode_memvalue(ulm_msg_p[i].value_1,
                                &config_dsp_usage_info_p->extern_d.num_1kb_blocks,
                                &config_dsp_usage_info_p->extern_d.percent_free);

            if (config_dsp_usage_info_p->extern_d.percent_free < config_dsp_usage_info_p->extern_d.min_percent_free) {
                config_dsp_usage_info_p->extern_d.min_percent_free = config_dsp_usage_info_p->extern_d.percent_free;
            }

             LOGMSG2("%s: Sample %d Mid %d Value is %x External Data %d 1KB Blocks %3.1f%% Free", __func__, i, mid, 
                    ulm_msg_p[i].value_1,
                    config_dsp_usage_info_p->extern_d.num_1kb_blocks,
                    config_dsp_usage_info_p->extern_d.percent_free);  
        }

        /* Current message is ULM_MEM_IN_DATA_ONLY */
        if (  (ulm_msg_p[i].ulm_pk_type == ULM_MEM_IN_DATA_ONLY) 
           && (ulm_msg_p[i].value_1_valid == true)) {

            stp_decode_memvalue(ulm_msg_p[i].value_1,
                                &config_dsp_usage_info_p->intern_d.num_1kb_blocks,
                                &config_dsp_usage_info_p->intern_d.percent_free);

            if (config_dsp_usage_info_p->intern_d.percent_free < config_dsp_usage_info_p->intern_d.min_percent_free) {
                config_dsp_usage_info_p->intern_d.min_percent_free = config_dsp_usage_info_p->intern_d.percent_free;
            }

             LOGMSG2("%s: Sample %d Mid %d Value is %x Internal Data %d 1KB Blocks %3.1f%% Free", __func__, i, mid, 
                    ulm_msg_p[i].value_1,
                    config_dsp_usage_info_p->intern_d.num_1kb_blocks,
                    config_dsp_usage_info_p->intern_d.percent_free);  
        }

        /* Current message is ULM_TEMP */
        if (  (ulm_msg_p[i].ulm_pk_type == ULM_TEMP) 
           && (ulm_msg_p[i].value_1_valid == true)) {
            config_dsp_usage_info_p->temp = ulm_msg_p[i].value_1;

             LOGMSG2("%s: Sample %d Mid %d Temperature %d C", __func__, i, mid, config_dsp_usage_info_p->temp); 
        }

    } /* End of sample processing for loop */

    /* For total mode need to pull run/idle time forward for all dsps to last sample.
     * Don't need to do this for MVA mode since it's taken care of on the front end
     * of the run/idle time processing.
     * Also note that if a dsp has generated an exit message, no run/idle time is added. 
    */
    if (config_mva_window == 0) {
        for ( int i = 0; i  < config_num_dsps; i++) {

            /* Skip since the last sample has alrady been processed */
            if ( ulm_msg_p[ulm_msg_cnt-1].master_id == i) {
                continue;
            }

            /* If first_valid_sample for the dsp is still true then add the 
             * elapsed time to run or idle time (since this has not been done yet).
             */
            if ((first_valid_sample[i] == true) && (run_time_flag[i] == true)) {
                /* run_tv_p[i] +=  elapsed_tv */
                sum_tv_to_tv(run_tv_p[i], elapsed_tv);
                LOGMSG2("%s: pull forward  DSP_%d run time with elapsed time to %d.%06d", __func__, i,
                        run_tv_p[i].tv_sec, run_tv_p[i].tv_usec);
            }
            if ((first_valid_sample[i] == true) && (idle_time_flag[i] == true)) {
                /* run_tv_p[i] +=  elapsed_tv */
                sum_tv_to_tv(idle_tv_p[i], elapsed_tv);
                LOGMSG2("%s: pull forward  DSP_%d idle time with elapsed time to %d.%06d", __func__, i,
                        idle_tv_p[i].tv_sec, idle_tv_p[i].tv_usec);
            }

            /* Plot usage */
            if (plot_usage_enable && (first_valid_sample[i] == true) && (run_time_flag[i] || idle_time_flag[i])) {
                ulm_plot_usage(i, &elapsed_tv);
            }    
            /* Now pull forward difference between the last sample and previous sample
             * for each core.
             */

            struct timeval last_tv_adj;
            struct timeval diff_tv;
            if (previous_tv[i].tv_usec > ulm_msg_p[ulm_msg_cnt-1].tv.tv_usec) {
                last_tv_adj.tv_usec = ulm_msg_p[i].tv.tv_usec + 1000000;
                last_tv_adj.tv_sec = ulm_msg_p[i].tv.tv_sec - 1;
                
            } else {
                last_tv_adj = ulm_msg_p[ulm_msg_cnt-1].tv;
            }
            diff_tv.tv_usec = last_tv_adj.tv_usec - previous_tv[i].tv_usec;
            diff_tv.tv_sec = last_tv_adj.tv_sec - previous_tv[i].tv_sec;        

            if (run_time_flag[i] == true) {
                /* run_tv_p[mid] +=  delta_tv */
                sum_tv_to_tv(run_tv_p[i], diff_tv);
                LOGMSG2("%s: pull forward  DSP_%d run time from last previous to %d.%06d", __func__, i,
                        run_tv_p[i].tv_sec, run_tv_p[i].tv_usec);
            }

            if (idle_time_flag[i] == true) {
                /* run_tv_p[mid] +=  delta_tv */
                sum_tv_to_tv(idle_tv_p[i], diff_tv);
                LOGMSG2("%s: pull forward  DSP_%d idle time from last previous to %d.%06d", __func__, i,
                        idle_tv_p[i].tv_sec, idle_tv_p[i].tv_usec);
            }

            if (plot_usage_enable && ((run_time_flag[i] || idle_time_flag[i]))) {
                ulm_plot_usage(i, &diff_tv);
            }    

        }
    }    

    /* Completed processing the buffer */
    free(ulm_msg_p);
    mva_first_sample = false;

    /* Update the delta gap for the next call to ulm_decode_usage() */
    double delta_gap = gap_end - gap_start;
    double delta_gap_sec;
    delta_gap_tv.tv_usec = (long)(modf(delta_gap, &delta_gap_sec) * 1e6);
    delta_gap_tv.tv_sec = (long)delta_gap_sec;

    /* if MVA provide accumulated run and idle time over all mva samples */
    if (config_mva_window != 0) {
        mva_total_usage();
        LOGMSG2("%s: MVA mode - Ending index is %d", __func__, mva_index * config_num_dsps);
    }

    /* Log the current usage for all dsps and count the number of 
     * dsps that are in the exit state.
     */
    int dsp_exit_cnt = 0;
    for (int i = 0; i < config_num_dsps; i++) {
        LOGMSG("%s: DSP_%d  run time %d.%06d  idle time %d.%06d", __func__, i,
              config_dsp_usage_info_p->run_tv_p[i].tv_sec,
              config_dsp_usage_info_p->run_tv_p[i].tv_usec, 
              config_dsp_usage_info_p->idle_tv_p[i].tv_sec,
              config_dsp_usage_info_p->idle_tv_p[i].tv_usec);

        dsp_exit_cnt += (exit_flag[i] == true) ? 1 : 0;

    }

    /* If all dsps are in the exit state then plot if enabled */
    if (dsp_exit_cnt == config_num_dsps) {
        LOGMSG("%s: All processors halted - if plotting enabled post process and launch plot", __func__);
        plot_postprocess(); 
        plot_launch();
    }

    /* Free malloced buffers */
    free(etb_buf_p);
    stp_decode_close();
}


/***************************************************************************** 
 *  ulm_advance_usage
 *
 *  Advance usage structure data in time
 *
 *  If recording_stopped is true then time advanced is added to advance_time.
 * 
 *  NOTE: If recording_stopped is false, previous_tv will be set to NULL(not used)  
 *  AND the input value for newtime will be discarded (re-calculated).
 *
 *****************************************************************************/
void ulm_advance_usage(double newtime, double current, struct timeval * previous_tv, bool recording_stopped)
{
    LOGFUNC();

    double dsp_delta_time[config_num_dsps];

    if (!recording_stopped) {
        double delta_time;
        struct timeval newtime_tv;
        newtime = gettime(newtime_tv);
        double total_recording_time = newtime - recording_start;

        /* Must add accumulated sync time  if any */
        total_recording_time += sync_recording_time;

        /* Just need to know how much time we have not added in yet */
        delta_time = total_recording_time - advance_time;
        advance_time += delta_time;

        for ( int j = 0; j  < config_num_dsps; j++) {
            dsp_delta_time[j] = delta_time;
        }

        LOGMSG("%s:Recording not stopped, total recording time is %f, new advance time is %f", __func__, total_recording_time, delta_time);

    } else {

        /* In this case the delta time will be different for each dsp */

        LOGMSG("%s:Recording stopped, elapsed time is %f, current time is %f", __func__, newtime, current);

        for ( int j = 0; j  < config_num_dsps; j++) {
            double previous = previous_tv[j].tv_usec/1e6 + previous_tv[j].tv_sec;
            /* Calculate the new delta time to the current sample. 
             * In this case newtime is the elapsed time (if first sample
             * recording dead time + recording gap time, otherwise 0. 
             */
            dsp_delta_time[j] = newtime + (current - previous);

            if (dsp_delta_time[j] < 0) {
                dsp_delta_time[j] = 0;
            }

            LOGMSG("%s:Recording stopped, DSP_%d previous is %f, advance time is %f", __func__, j, previous, dsp_delta_time[j]); 
        }
  
    }
        
    /* Initialize run and idle time sample pointers for the mode */
    struct timeval * run_tv_p = config_dsp_usage_info_p->run_tv_p;
    struct timeval * idle_tv_p = config_dsp_usage_info_p->idle_tv_p;

    if (config_mva_window != 0) {

        /* MVA mode - so if not the first time, increment the run and idle time sample mva_index
         * to the next, avoid writing over data from the previous session.
         */

        run_tv_p = mva_run_tv_p + (mva_index * config_num_dsps);
        idle_tv_p = mva_idle_tv_p + (mva_index * config_num_dsps);

        LOGMSG2("%s: MVA mode - Starting mva index is %d", __func__, mva_index * config_num_dsps);
    }

    /* Don't advance if no data detected for this dsp */
    int done = 0;
    for ( int j = 0; j  < config_num_dsps; j++) {

        if ((mva_index == 0) && (!run_time_flag[j]) && (!idle_time_flag[j])) {
            done++;
        }
    }
    if (done >= config_num_dsps) {
        return;
    }
    done = 0;

    if (plot_usage_enable) {

        for ( int j = 0; j  < config_num_dsps; j++) {

            if ((run_time_flag[j] || idle_time_flag[j])) {

                struct timeval delta_tv;
                double delta_sec;
                delta_tv.tv_usec  = (long)(modf(dsp_delta_time[j], &delta_sec) * 1e6);
                delta_tv.tv_sec = (long)delta_sec;

                ulm_plot_usage(j, &delta_tv);
            }   
        }
    }

    if (config_mva_window != 0) {

        double threshold_delta[config_num_dsps]; 

        double mva_threshold = (mva_sample_rate == 10) ? 0.1 : 1.0;
        LOGMSG2("%s: mva threshold is %f", __func__, mva_threshold);
 
        for ( int j = 0; j  < config_num_dsps; j++) {

            double runtime = run_tv_p[j].tv_usec/1e6 + run_tv_p[j].tv_sec;
            double idletime = idle_tv_p[j].tv_usec/1e6 + idle_tv_p[j].tv_sec;

            threshold_delta[j] = mva_threshold - (runtime + idletime);

            /* If time left does not cross the threshold then increment done */
            if (threshold_delta[j] > dsp_delta_time[j]) {
                threshold_delta[j] = dsp_delta_time[j];
                done++;
            }

            dsp_delta_time[j] = dsp_delta_time[j] - threshold_delta[j];

            double threshold_delta_sec;
            double threshold_delta_usec  = modf(threshold_delta[j], &threshold_delta_sec) * 1e6;

            LOGMSG2("%s: DSP_%d runtime %f idle time %f MVA Threshold delta %f dsp_delta %f", __func__, j, runtime, idletime, threshold_delta[j], dsp_delta_time[j]);

            if (run_time_flag[j] == true) {

                sum_double_to_tv(run_tv_p[j], threshold_delta_sec, threshold_delta_usec);

                LOGMSG2("%s: DSP_%d MVA Threshold advance run time %d.%06d", __func__, j,
                        run_tv_p[j].tv_sec, run_tv_p[j].tv_usec);
            } 

            if (idle_time_flag[j] == true) {

                sum_double_to_tv(idle_tv_p[j], threshold_delta_sec, threshold_delta_usec);  

                LOGMSG2("%s: DSP_%d MVA Threshold advance idle time %d.%06d", __func__, j,
                        idle_tv_p[j].tv_sec, idle_tv_p[j].tv_usec);
            }

            if (!run_time_flag[j] && !idle_time_flag[j] && !exit_flag[j]) {

                clear_tv(idle_tv_p[j]);
                clear_tv(run_tv_p[j]);

                LOGMSG2("%s: DSP_%d Threshold clear run and idle time", __func__, j);                                 
            }

            if ((previous_tv != NULL) && ((run_time_flag[j] == true) || (idle_time_flag[j] == true))) {

                sum_double_to_tv(previous_tv[j], threshold_delta_sec, threshold_delta_usec);
            }

        } /* End of for */

        int mva_cnt = 0;        
        do {

            /* If the done counter is == the number of DSPs or every mva sample has been updated once */
            if ((done >= config_num_dsps) || (mva_cnt++ == mva_num_tv_samples/config_num_dsps)) {
                break;
            }

            /* If all DSPs not done - try again */
            if (done != config_num_dsps) {
                done = 0;
            }

            /* check if buffer circulated */
            mva_index++;
            if ((mva_index * config_num_dsps) == mva_num_tv_samples) { 
                run_tv_p = mva_run_tv_p;
                idle_tv_p = mva_idle_tv_p;
                mva_index = 0;
                mva_wrapped = true;
            } else {
                /* Increment tameval samples by the number of DSPs in the Device */
                run_tv_p += config_num_dsps;
                idle_tv_p += config_num_dsps;
            }

            LOGMSG2("%s: MVA advance - new mva_index is %d, done count is %d", __func__, mva_index * config_num_dsps, done);

            /* Clear the idle and run time for each dsp - Just in case dsp_delta_time is 0 */
            for (int j = 0; j  < config_num_dsps; j++) { 
                if (!exit_flag[j]) {
                    clear_tv(idle_tv_p[j]);
                    clear_tv(run_tv_p[j]);

                    LOGMSG2("%s: DSP_%d MVA initialize run and idle time to 0", __func__, j);

                }
            }

            /* For each DSP - if advancing more than 0, adjust skipped samples to reflect
             * 100% run or 100% idle or 0 if no run or idle.
             */

            for (int j = 0; j  < config_num_dsps; j++) {

                if (dsp_delta_time[j] >= mva_threshold) {

                    if (run_time_flag[j] == true) {

                        run_tv_p[j].tv_usec = (mva_threshold == .1) ? 100000 : 0;
                        run_tv_p[j].tv_sec = (mva_threshold == .1) ? 0 : 1;

                        LOGMSG2("%s: DSP_%d MVA advance run time %d.%06d", __func__, j,
                                run_tv_p[j].tv_sec, run_tv_p[j].tv_usec);

                        clear_tv(idle_tv_p[j]);
                    }

                    if (idle_time_flag[j] == true) {

                        idle_tv_p[j].tv_usec = (mva_threshold == .1) ? 100000 : 0;
                        idle_tv_p[j].tv_sec = (mva_threshold == .1) ? 0 : 1;

                        LOGMSG2("%s: DSP_%d MVA advance idle time %d.%06d", __func__, j,
                                idle_tv_p[j].tv_sec, idle_tv_p[j].tv_usec);

                        clear_tv(run_tv_p[j]);

                    }

                    if (!run_time_flag[j] && !idle_time_flag[j] && !exit_flag[j]) {

                        clear_tv(idle_tv_p[j]);
                        clear_tv(run_tv_p[j]);

                        LOGMSG2("%s: DSP_%d MVA clear run and idle time", __func__, j);                                 

                    }

                    if ((previous_tv != NULL) && ((run_time_flag[j] == true) || (idle_time_flag[j] == true))) {

                        previous_tv[j].tv_usec += (mva_threshold == .1) ? 100000 : 0;;
                        previous_tv[j].tv_sec += (mva_threshold == .1) ? 0 : 1;;

                        if (previous_tv[j].tv_usec >= 1000000) {
                            previous_tv[j].tv_usec -= 1000000;
                            previous_tv[j].tv_sec += 1;
                        }
                    }

                    dsp_delta_time[j] -= mva_threshold;

                } else {
                    done++;
                }

            } /* End of for */

        } while (1);   /* End of advance for loop*/

        /* Initialize next run and idle time with leftover time */
        for ( int j = 0; j  < config_num_dsps; j++) {   

                double leftover_time = dsp_delta_time[j];

                if (leftover_time > 0) {

                    LOGMSG2("%s: DSP_%d MVA delta leftover time %f", __func__, j, leftover_time);

                    double adv_sec = 0;
                    double adv_usec = modf(leftover_time, &adv_sec) * 1e6;

                    if (run_time_flag[j]) {
                        run_tv_p[j].tv_usec = (long)adv_usec;
                        run_tv_p[j].tv_sec = (long)adv_sec;

                        LOGMSG2("%s: DSP_%d MVA apply leftover to run time %d.%06d", __func__, j, 
                               (long)adv_sec, (long)adv_usec);

                        clear_tv(idle_tv_p[j]);
                    }

                    if (idle_time_flag[j]) {
                        idle_tv_p[j].tv_usec = (long)adv_usec;
                        idle_tv_p[j].tv_sec = (long)adv_sec;

                        LOGMSG2("%s: DSP_%d MVA apply leftover to idle time %d.%06d", __func__, j, 
                               (long)adv_sec, (long)adv_usec);

                        clear_tv(run_tv_p[j]);
                    }

                    if (!run_time_flag[j] && !idle_time_flag[j] && !exit_flag[j]) {

                        clear_tv(idle_tv_p[j]);
                        clear_tv(run_tv_p[j]);
                    }

                    if ((previous_tv != NULL) && ((run_time_flag[j] == true) || (idle_time_flag[j] == true))) {

                        sum_double_to_tv(previous_tv[j], adv_sec, adv_usec);

                        LOGMSG2("%s: DSP_%d new previous_tv is %d.%06d", __func__, j, 
                               previous_tv[j].tv_sec, previous_tv[j].tv_usec);

                    }
                }

        } /* End of for */
        
        if (!recording_stopped) {
            mva_total_usage();
        }

    } else {

        /* Advance for Total Mode */
        for ( int j = 0; j  < config_num_dsps; j++) { 
            
            double adv_sec = 0;
            double adv_usec = modf(dsp_delta_time[j], &adv_sec) * 1e6;

            if (run_time_flag[j]) {

                sum_double_to_tv(run_tv_p[j], adv_sec, adv_usec);

                LOGMSG2("%s: DSP_%d Total mode apply delta time %d.%06d - new run time %d.%06d", __func__, j, 
                       (long)adv_sec, (long)adv_usec, run_tv_p[j].tv_sec, run_tv_p[j].tv_usec);
            }
            if (idle_time_flag[j]) {

                sum_double_to_tv(idle_tv_p[j], adv_sec, adv_usec);

                LOGMSG2("%s: DSP_%d Total mode apply delta time %d.%06d - new idle time %d.%06d", __func__, j, 
                       (long)adv_sec, (long)adv_usec, idle_tv_p[j].tv_sec, idle_tv_p[j].tv_usec);   
            }
            if (!run_time_flag[j] && !idle_time_flag[j] && !exit_flag[j]) {

                clear_tv(idle_tv_p[j]);
                clear_tv(run_tv_p[j]);
            }
            if ((previous_tv != NULL) && ((run_time_flag[j] == true) || (idle_time_flag[j] == true))) {

                sum_double_to_tv(previous_tv[j], adv_sec, adv_usec);
            }
        } /* End of for */
    }
}

/***************************************************************************** 
 *  ulm_log_lastusage
 *
 *****************************************************************************/
void ulm_log_lastusage()
{
    LOGMSG3("Num dsps is %d", config_num_dsps);

    for (int i = 0; i < config_num_dsps; i++) {
        LOGMSG3("DSP_%d run time %d.%06d idle time %d.%06d", i,
              config_dsp_usage_info_p->run_tv_p[i].tv_sec,
              config_dsp_usage_info_p->run_tv_p[i].tv_usec, 
              config_dsp_usage_info_p->idle_tv_p[i].tv_sec,
              config_dsp_usage_info_p->idle_tv_p[i].tv_usec);
    }
}

/***************************************************************************** 
 *  ulm_get_gaptime
 *
 *****************************************************************************/
void ulm_get_gaptime(struct timeval * gap_tv_p)
{
    LOGFUNC();

    double gap_integral;
    gap_tv_p->tv_usec = (long)(modf(gap, &gap_integral) * 1e6);
    gap_tv_p->tv_sec = (long)gap_integral;

}

/***************************************************************************** 
 *  ulm_clr_gaptime
 *
 *****************************************************************************/
void ulm_clr_gaptime()
{
    gap = 0;
}


/***************************************************************************** 
 *  ulm_get_etb_state
 *
 *****************************************************************************/
void ulm_get_etb_state(int * etb_percent_full, bool * is_etb_enabled, bool * is_etb_wrapped, bool * is_logging, bool * stop_trigger)
{
    LOGFUNC();

    int etb_bytes_avaiable = 0;

    /* Check if etb has been configured */
    if (config_etb_size == 0) {
        *etb_percent_full = 0;
        *is_etb_enabled = false;
        *is_etb_wrapped = false;
        *is_logging = false;
        *stop_trigger = false;
        return;    
    }

    *is_logging = config_logging_mode;

    etb_bytes_avaiable = etb_status(is_etb_enabled, is_etb_wrapped) * 4;

    *etb_percent_full = (etb_bytes_avaiable * 100)/config_etb_size;

    /* Set the percent full to 1 if there is at least 1 statemsg 
     * packet or 3 state packets in the ETB. This will allow the
     * signal handler to decide to decode after a threshold time
     * has elapsed with no data decoded (keeps data from getting
     * stuck in the ETB without being displayed).
     */
    if ((*etb_percent_full == 0) && (etb_bytes_avaiable > 29)) {
       *etb_percent_full = 1; 
    }

    if (config_logging_mode && config_stop_full && (etb_bytes_avaiable == config_etb_size)) {
        *stop_trigger = true;
    } else {
        *stop_trigger = false;
    }

}

/***************************************************************************** 
 *  ulm_generate_testdata
 *
 *  Issue ulm messages.
 *
 *****************************************************************************/
void ulm_generate_testdata()
{
    LOGFUNC();

    for (int i = 0; i < 16; i++) {
        if (i & 1){ 
            ulm_put_state(ULM_STATE_IDLE);
        } else {
            ulm_put_state(ULM_STATE_RUN);
        }
    }

    ulm_put_temp(-10);
    ulm_put_temp(10);

    usleep(2500);

    for (int i = 0; i < 8; i++) {
        if (i & 1){ 
            ulm_put_state(ULM_STATE_IDLE);
        } else {
            ulm_put_state(ULM_STATE_RUN);
        }
    }

    for (int i = 0; i < 16; i++) {
        if (i & 1){ 
            ulm_put_state(ULM_STATE_IDLE);
        } else {
            ulm_put_state(ULM_STATE_RUN);
        }
    }

    ulm_put_statemsg(ULM_STATE_IDLE, 0x01234567, 0x89abcdef);
    ulm_put_statemsg(ULM_STATE_RUN, 0x76543210, 0xfedcba98);

    for (int i = ULM_OMP_EVENT_PARALLEL_START_ENTER; i < ULM_STATE_EXIT; i++) {
        ulm_put_state(i);
        ulm_put_statemsg(i, 100+i, 1000+i);
    }

    for (int i = 0; i < 8; i++) {
        ulm_put_state(ULM_STATE_EXIT);
        ulm_put_statemsg(ULM_STATE_EXIT, 0x1, 0x2);
    }    

    ulm_put_temp(-20);
    ulm_put_temp(20);

    ulm_put_mem(ULM_MEM_EX_CODE_AND_DATA, 256, .973);
    ulm_put_mem(ULM_MEM_EX_DATA_ONLY, 65537, .874);
    ulm_put_mem(ULM_MEM_IN_DATA_ONLY, 128, .33599);

    ulm_put_temp(0);
}

/***************************************************************************** 
 *  ulm_generate_testlarge
 *
 *  Issue ulm messages.
 *
 *****************************************************************************/
void ulm_generate_testlarge()
{
    LOGFUNC();

    for (int i = 0; i < 4000; i++) {

        if (i & 1){ 
            ulm_put_state(ULM_STATE_IDLE);
        } else {
            ulm_put_state(ULM_STATE_RUN);
        }
    }

    for (int i = 0; i < config_num_dsps; i++) {
       ulm_put_state(ULM_STATE_EXIT);    
    }

}

/***************************************************************************** 
 *  ulm_generate_testmsg
 *
 *  Issue ulm messages.
 *
 *****************************************************************************/
static long test_msg_cnt = 0;

void ulm_generate_testmsg()
{
    LOGFUNC();

    int index = test_msg_cnt % ULM_STATE_LAST;    
    ulm_state_t state = ulm_get_runstate(index);

    /* Skip EXIT messages - must do this in a while since 
     * there are more than one ulm EXIT state message.
     */ 
    while (state == ULM_STATE_EXIT) {                
        test_msg_cnt++;
        index = test_msg_cnt % ULM_STATE_LAST;
        state = ulm_get_runstate(index);
    }

    for (int i = 0; i < config_num_dsps; i++) {
        ulm_put_statemsg(index, test_msg_cnt, test_msg_cnt);    

    }

    if (test_msg_cnt == 100) {
        ulm_put_mem(ULM_MEM_EX_CODE_AND_DATA, 256, .973);
        ulm_put_mem(ULM_MEM_EX_DATA_ONLY, 65537, .874);
        ulm_put_mem(ULM_MEM_IN_DATA_ONLY, 128, .33599);
    }

    if (test_msg_cnt == 400) {
        ulm_put_mem(ULM_MEM_EX_CODE_AND_DATA, 128, .903);
        ulm_put_mem(ULM_MEM_EX_DATA_ONLY, 32768, .804);
        ulm_put_mem(ULM_MEM_IN_DATA_ONLY, 64, .30599);
    }

    if (test_msg_cnt > 500) {
        for (int i = 0; i < config_num_dsps; i++) {
            ulm_put_statemsg(ULM_STATE_EXIT, test_msg_cnt, test_msg_cnt);    
        }
        test_msg_cnt = 0;
    }


    if (!(test_msg_cnt % 100)) {
        LOGMSG2("%s: Test message count is %d", __func__, test_msg_cnt);
    }

    test_msg_cnt++;

}

/******************************************************************************
 * Private Functions
 *****************************************************************************/

static char * ulm_error_msg[] = { 
    [ULM_ERR_STM_OWNERSHIP_NOT_GRANTED] = "STM Ownership Not Granted",
    [ULM_ERR_STM_FLUSH_FAILED] = "STM Flush Failed",
    [ULM_ERR_STM_MMAP] = "STM mmap failure"
};

/***************************************************************************** 
 *  ulm_error_handler
 *
 *****************************************************************************/
static inline void ulm_error_handler(ulm_error_t error)
{

    char * msg = ulm_error_msg[error];
    err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);

}

/***************************************************************************** 
 *  ulm_totalmva_usage
 *
 *****************************************************************************/
void mva_total_usage()
{

    for (int j = 0; j < config_num_dsps; j++) {
        config_dsp_usage_info_p->run_tv_p[j].tv_sec = 0;
        config_dsp_usage_info_p->run_tv_p[j].tv_usec = 0;
        config_dsp_usage_info_p->idle_tv_p[j].tv_sec = 0;
        config_dsp_usage_info_p->idle_tv_p[j].tv_usec = 0;
    }

    for (int i = 0; i < (mva_num_tv_samples/config_num_dsps); i++) {
        for (int j = 0; j < config_num_dsps; j++) {

            config_dsp_usage_info_p->run_tv_p[j].tv_usec += (mva_run_tv_p + (i*config_num_dsps))[j].tv_usec;
            if (config_dsp_usage_info_p->run_tv_p[j].tv_usec >= 1000000) {
                config_dsp_usage_info_p->run_tv_p[j].tv_usec -= 1000000;
                config_dsp_usage_info_p->run_tv_p[j].tv_sec++;
            }
            config_dsp_usage_info_p->run_tv_p[j].tv_sec += (mva_run_tv_p + (i*config_num_dsps))[j].tv_sec;

            config_dsp_usage_info_p->idle_tv_p[j].tv_usec += (mva_idle_tv_p + (i*config_num_dsps))[j].tv_usec;
            if (config_dsp_usage_info_p->idle_tv_p[j].tv_usec >= 1000000) {
                config_dsp_usage_info_p->idle_tv_p[j].tv_usec -= 1000000;
                config_dsp_usage_info_p->idle_tv_p[j].tv_sec++;
            }
            config_dsp_usage_info_p->idle_tv_p[j].tv_sec += (mva_idle_tv_p + (i*config_num_dsps))[j].tv_sec;
        }
    }
}


/***************************************************************************** 
 *  ulm_plot_usage()
 *
 *****************************************************************************/
static void ulm_plot_usage(int dsp_index, struct timeval * delta_tv_p)
{
    LOGFUNC();

    double delta_time = delta_tv_p->tv_usec/1e6 + delta_tv_p->tv_sec;

    LOGMSG("%s: plot delta time is %f", __func__, delta_time);

    do {
        if ((plot_runtime[dsp_index] + plot_idletime[dsp_index] + delta_time) >= plot_resolution) {

            double diff_time = plot_resolution - (plot_runtime[dsp_index] + plot_idletime[dsp_index]);
            plot_runtime[dsp_index] += (run_time_flag[dsp_index]) ? diff_time : 0;
            plot_idletime[dsp_index] += (idle_time_flag[dsp_index]) ? diff_time : 0;
            plot_timestamp[dsp_index] += plot_resolution;

            plot_write_current(dsp_index, plot_timestamp[dsp_index], plot_runtime[dsp_index], plot_idletime[dsp_index], config_dsp_usage_info_p->temp);

            plot_runtime[dsp_index] = 0;
            plot_idletime[dsp_index] = 0;
            delta_time -= (delta_time > diff_time) ? diff_time : delta_time;

        } 

        if ((delta_time > 0) && ((plot_runtime[dsp_index] + plot_idletime[dsp_index] + delta_time) < plot_resolution)) {
            plot_runtime[dsp_index] += (run_time_flag[dsp_index]) ? delta_time : 0;
            plot_idletime[dsp_index] += (idle_time_flag[dsp_index]) ? delta_time : 0;

            delta_time = 0;
        }

    } while (delta_time > 0);
}

/***************************************************************************** 
 *  ulm_plot_clear()
 *
 *****************************************************************************/
static void ulm_plot_clear()
{
    LOGFUNC();

    for (int j = 0; j < config_num_dsps; j++) {
        plot_timestamp[j] = 0;
        plot_runtime[j] = 0;
        plot_idletime[j] = 0;
    }    

}

