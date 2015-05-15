/*
 * signal_handler.c
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
#include <signal.h>
#include <unistd.h>

#include "error_handler.h"
#include "debug_log.h"
#include "dsptop.h"
#include "signal_handler.h"
#include "io.h"
#include "ulm_handler.h"
#include "tiulm.h"
#include "temp.h"
#include "utility.h"

struct sigaction action;


/* SIGALRM is every 5 milliseconds - all other time based functions
 * are dirived from this value.
 */

/* Set temperature logging rate at 1 sample every .5 seconds. */
static int temp_rate = 100;
static int temp_cnt = 100;

/* Set minimum decode rate at twice every second. */
const static int min_usage_decode_rate = 100;
static int min_usage_decode_cnt = 100; 

/* Set minimum time stamp sync every second.
 * Note: This keeps the STM timestamp counter from saturating 
 */
const static int sync_rate = 2000;
static int sync_cnt = 2000; 

/* Set slow test data rate at 1 sample every 5 seconds. */
static const int test_generation_rate = 1000;
static int test_generation_cnt = 1000;
bool first_slow_testmsg = true;
static int previous_etb_percent_full = 0;

/***************************************************************************** 
 *  Private Functions
 *****************************************************************************/
/***************************************************************************** 
 *  signal_handler()
 *
 *  This is a common signal handler for all signals
 *
 *  - If the signal is not expected then generate a fatal error and exit.
 *  - Handle the signal.
 *    - If SIGWINCH resize the window.
 *    - if SIGINT and recording then stop recording.
 *    - If SIGPIPE then broken pipe so close the pipe.
 *  
 *****************************************************************************/
void signal_handler(int sig) {

    LOGFUNC();

    LOGMSG("%s:Caught signal %d", __func__, sig); 

    /* Handle the signal */
    switch (sig) {
    case SIGALRM:
    {
        int etb_percent_full = 0;
        bool is_etb_enabled = false;
        bool is_etb_wrapped = false;
        bool is_logging = false;
        bool stop_trigger = false;  /* Only set if logging mode and etb stop on full set */

        ulm_get_etb_state(&etb_percent_full, &is_etb_enabled, &is_etb_wrapped, &is_logging, &stop_trigger);

        LOGMSG("%s: SIGALRM - etb enabled %d, percent full %d, is wrapped %d, is logging %d, is stop trigger %d", 
                __func__, is_etb_enabled, etb_percent_full, is_etb_wrapped, is_logging, stop_trigger);

        /* Issue test mode message */
        if ((is_etb_enabled) && (g_mva_test_mode == TEST_MODE_TIME_FAST)) {
                ulm_generate_testmsg();
        }

        if ((is_etb_enabled) && (g_mva_test_mode == TEST_MODE_TIME_SLOW)) {
            if ((--test_generation_cnt == 0) || first_slow_testmsg) {
                ulm_generate_testmsg();
                test_generation_cnt = test_generation_rate;
                first_slow_testmsg = false;
            }
        }

        if (stop_trigger) {
            term();
        }

        /* Get Temperature 
         * Note - Temperature is simply written every .5 seconds. You may think that
         * the temperature samples should only be updated when there is a temperature
         * change (and one last value when message collection is terminated), but you
         * would be wrong. There are two cases the ETB capture causes that makes that
         * kind of algorithm problematic:
         * 1 ) ETB is in stop when full mode - can't write last temperature value in buffer
         *     (because ETB capture stops) and you may only have one temperature value at
         *     the start of the buffer.
         * 2 ) ETB is in circular mode - since the buffer may have wrapped many times, you
         *     may end up with only a single temperature value at the end of the data. When
         *     logging no idea when the buffer wraps.
         */
        if (temp_is_enabled() && (--temp_cnt == 0)) {
            /* Note - call to temp_get will set usage temp.
             *        Just using return value for logging.
             */
            int8_t temp = temp_get();
            if (is_logging) {
                ulm_put_temp(temp);
            }

	        temp_cnt = temp_rate;

        }

        /* Check if sync message needed to keep STM timestamp counter from saturating */
        if (--sync_cnt == 0) {
            ulm_put_sync();
            sync_cnt = sync_rate;
        }

        /* Check if decoding service required */
        if (is_etb_enabled && !is_logging) {

            --min_usage_decode_cnt;
            if ((etb_percent_full == 0) && (min_usage_decode_cnt == 0)){
 
                bool recoding_stopped = false;
                ulm_advance_usage(0, 0, NULL, recoding_stopped);
            }

            /* Stop collection and decode if etb is half full, 
             * or the there is no new data since the last check,
             * or minimum decode rate count (.5 sec) has expired.
             */ 
            if (  (etb_percent_full > 50)
               || ((etb_percent_full > 0) && (previous_etb_percent_full == etb_percent_full))
               || ((etb_percent_full > 0) && (min_usage_decode_cnt == 0))) {

                ulm_decode_usage();
                previous_etb_percent_full = 0;
            }
            else {
                previous_etb_percent_full = etb_percent_full;
            }

            if (min_usage_decode_cnt == 0) {
                min_usage_decode_cnt = min_usage_decode_rate;
            }

        }

        sigaction(SIGALRM, &action, NULL);           
        break;
    }
    case SIGINT:
    {
        int etb_percent_full = 0;
        bool is_etb_enabled = false;
        bool is_etb_wrapped = false;
        bool is_logging = false;
        bool stop_trigger = false;  /* Only set if logging mode and etb stop on full set */

        ulm_get_etb_state(&etb_percent_full, &is_etb_enabled, &is_etb_wrapped, &is_logging, &stop_trigger);

        LOGMSG("%s: SIGINT - etb enabled %d, percent full %d, is wrapped %d, is logging %d, is stop trigger %d", 
                __func__, is_etb_enabled, etb_percent_full, is_etb_wrapped, is_logging, stop_trigger);

        /* Check if decoding service required */
        if (is_etb_enabled && !is_logging && (etb_percent_full > 0)) {
            ulm_decode_usage();
        }
        term();
        sigaction(SIGINT, &action, NULL);
        break;
    }
    case SIGPIPE:
        /* Client has broken the pipe - so simply stop using it */
        LOGMSG("%s:caught SIGPIP - pipe is broken", __func__);
        util_pipe_close();
        break;
    case SIGWINCH:
        LOGMSG("%s:caught SIGWINCH - window resized", __func__);
        sigaction(SIGWINCH, &action, NULL);
        global_state.resize_window_event = true;
        break;
    default:
    {
        char * msg = "caught invalid signal";
        LOGMSG("%s:%s %d", __func__, msg, sig);
        const size_t max_msg_size = 32;
        char err_msg[max_msg_size];
        snprintf(err_msg, max_msg_size, "%s %d", msg, sig);
        err_handler(ERR_TYPE_LOCAL, ERR_INVALID_SIGNAL_CAUGHT, err_msg);
    }
    } /* End of switch */

    LOGMSG("%s:Exit", __func__);
}

/***************************************************************************** 
 *  Public Functions
 *****************************************************************************/
/***************************************************************************** 
 *  signal_handler_init()
 *
 *  Initialize the signal handling
 *
 *  Note: This function takes the start_recording and stop_recording functions
 *  as inputs eliminating the need to include remote_commands.h. 
 *  
 *****************************************************************************/
void signal_handler_init()
{
    LOGFUNC();

    sigset_t block_mask;

    memset(&action, 0, sizeof(struct sigaction));

    /* Setup signals to be masked while servicing a signal*/
    sigemptyset(&block_mask);
    sigaddset(&block_mask, SIGALRM);
    sigaddset(&block_mask, SIGWINCH);
    sigaddset(&block_mask, SIGINT);

    action.sa_handler = signal_handler;
    action.sa_mask = block_mask;
    
    /*Register signal handlers */
    sigaction(SIGALRM, &action, NULL);
    sigaction(SIGWINCH, &action, NULL);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGPIPE, &action, NULL);

    if (g_mva_test_mode != TEST_MODE_DISABLED) {
        temp_rate = 100;
        temp_cnt = 100;
    } 

}

/***************************************************************************** 
 *  signal_handler_block()
 *
 *  Sets signals to block.
 *
 *  Note: This function saves the original signal blocking mask in orig_mask.
 *  
 *****************************************************************************/
static bool signals_blocked = false;
static sigset_t block_mask;

void signal_handler_block()
{

    if (signals_blocked == true) {
        return;
    }

    sigemptyset(&block_mask);
    sigaddset(&block_mask, SIGALRM);
    sigaddset(&block_mask, SIGWINCH);
    sigaddset(&block_mask, SIGINT);

	if (sigprocmask(SIG_BLOCK, &block_mask, NULL) < 0) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
	}

    signals_blocked = true;

    /* Don't want to call this until signals are blocked */
    LOGFUNC();

}
/***************************************************************************** 
 *  signal_handler_unblock()
 *
 *  Restores original signal state prior to blocking.
 *
 *  Note: Do not call this function without calling signal_handler_block() first.
 *  
 *****************************************************************************/
void signal_handler_unblock()
{
    LOGFUNC();

    if (signals_blocked == false) {
        return;
    }

    if (sigprocmask(SIG_UNBLOCK, &block_mask, NULL) < 0) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
	}

    signals_blocked = false;
    
}

