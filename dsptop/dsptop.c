/*
 * dsptop.c
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
#include <stdbool.h>
#include <string.h>
#include <getopt.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "error_handler.h"
#include "error.h"
#include "debug_log.h"
#include "signal_handler.h"

#include "dsptop.h"
#include "command.h"
#include "io.h"
#include "map.h"
#include "etb_handler.h"
#include "temp.h"
#include "ulm_handler.h"
#include "utility.h"
#include "version.h"
#include "plot.h"

#include "ncurses.h"
#include "tiulm.h"

/***************************************************************************** 
 *  Global declarations
 *****************************************************************************/

const int g_major_version = MAJOR_VERSION;
const int g_minor_version = MINOR_VERSION;
const int g_patch_version = PATCH_VERSION;
const int g_copyright_year = 2014;

FILE *g_stdout;
FILE *g_stderr;
char * g_whoami = NULL;
test_mode_t g_mva_test_mode;
int g_total_ulm_msgcnt = 0;

/* Global server state initialization*/
global_state_t global_state = {
    .resize_window_event = false,
    .etb_active = false,
    .etb_mode = ETB_MODE_ONESHOT_CIRCULAR,
    .recording = STATE_UNINITIALIZED
};

/***************************************************************************** 
 *  Private declarations
 *****************************************************************************/
/* dsp_type_table rules:
 * 1. DSPs must be populated in the name and id arrays first, followed by ARM cores.
 * 2. "Unknown" must be at the end of the table.
 */

char * master_names_66AK2Hxx[] = {"dsp_0", "dsp_1", "dsp_2", "dsp_3",
                                  "dsp_4", "dsp_5", "dsp_6", "dsp_7",
                                  "arm_0", "arm_1", "arm_2", "arm_3"};
uint8_t master_id_66AK2Hxx[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xA, 0xB};

char * master_names_TCI6630K2L[] = {"dsp_0", "dsp_1", "dsp_2", "dsp_3", "arm_0", "arm_1"};
uint8_t master_id_TCI6630K2L[] = {0, 1, 2, 3, 8, 9};

char * master_names_66AK2Exx[] = {"dsp_0", "arm_0", "arm_1", "arm_2", "arm_3"};
uint8_t master_id_66AK2Exx[] = { 0, 8, 9, 0xA, 0xB};

char * master_names_DRA74x[] = {"dsp_0", "arm_0", "arm_1"};
uint8_t master_id_DRA74x[] = {0x20, 0, 1};

char * master_names_DRA75x[] = { "dsp_0", "dsp_1", "arm_0", "arm_1"};
uint8_t master_id_DRA75x[] = { 0x20, 0x34, 0, 1};

char * master_names_AM571x[] = {"dsp_0", "arm_0"};
uint8_t master_id_AM571x[] = {0x20, 0};

char * master_names_AM572x[] = { "dsp_0", "dsp_1", "arm_0", "arm_1"};
uint8_t master_id_AM572x[] = { 0x20, 0x34, 0, 1};

static dsp_type_t dsp_type_table[] = {
    {.name = "66AK2Hxx", .num_dsps = 8, true, 12, master_id_66AK2Hxx, master_names_66AK2Hxx}, 
    {.name = "TCI6630K2L", .num_dsps = 4, true, 6, master_id_TCI6630K2L, master_names_TCI6630K2L},   
    {.name = "66AK2Exx", .num_dsps = 1, true, 5, master_id_66AK2Exx, master_names_66AK2Exx},
    {.name = "DRA74x", .num_dsps = 1, false, 3, master_id_DRA74x, master_names_DRA74x},
    {.name = "DRA75x", .num_dsps = 2, false, 4, master_id_DRA75x, master_names_DRA74x},
    {.name = "AM571x", .num_dsps = 1, false, 2, master_id_AM571x, master_names_AM571x},
    {.name = "AM572x", .num_dsps = 2, false, 4, master_id_AM572x, master_names_AM572x},
    {.name = "Unknown", .num_dsps = 8, false, 0, NULL, NULL}   
};


/* Initialize dsptop operating parameter defaults */
static options_t options = {
    .out_filename_p = NULL,
    .format_type = FMT_TEXT,
    .max_filesize = 0,
    .logging_enable = false,
    .sync_enable = false,
    .stop_full = false,
    .quiet_mode = false,
    .process_until_exitmsg = false,
    .num_iterations = 0,
    .delay_rate = 3.0,
    .mva_window = 3.0,
    .test_mode = TEST_MODE_DISABLED,
    .plot_resolution = .1
};

/*Note: dsptop_info elapsed_tv and gap_tv elements are
 * initialized by clr_usage_values().
 */
static dsptop_info_t dsptop_info = {
        .dsptop_mode = MOVING_AVERAGE_MODE
}; 

static struct timeval display_rate_tv;
static struct timeval initial_tv;
static struct timeval previous_tv;
static struct timeval * initial_tv_p = NULL;

static dsp_usage_info_t dsp_usage_info;

const long wait_time_quiet_usec = 10000;
const long wait_time_timer_usec = 5000;

/***************************************************************************** 
 *  Private function prototypes
 *****************************************************************************/

static dsp_type_t * get_dspinfo();
static void get_elapsedtime(struct timeval * elapsed_tv, struct timeval * delta_tv, bool reset_delta);
static void set_itimer();
void clr_usage_values(dsptop_info_t * dsptop_info_p);
static void eval_cmd(interactive_cmd_t cmd);

/***************************************************************************** 
 *  Public functions
 *****************************************************************************/

/***************************************************************************** 
 *  main()
 *
 *  Main evaluates a set of simple command line parameters and then drops into
 *  an endless loop that processes interactive commands or process trace data 
 *  for the load display or logging.  
 * 
 *****************************************************************************/
 
int main(int argc, char *argv[])
{
    /* Initialize globals */
    g_stdout = stdout;
    g_stderr = stderr;
    g_whoami = strdup(argv[0]);
    g_mva_test_mode = TEST_MODE_DISABLED;

    /* Initialize filenames */
    const size_t filename_max_size = 256;
    char rc_filename[filename_max_size];
    char fifo_filename[filename_max_size];

    /* Check TI_LOGGING environment variable */
    char * ti_trace_logging_p = getenv("TI_TRACE_LOGGING");
    if (ti_trace_logging_p != NULL) {
        int  debug_log_level = atoi(ti_trace_logging_p);
        if (debug_log_level > 0) {
            debug_log_int(debug_log_level);
            LOGMSG("%s:Debug logging to dsptop_log.txt enabled with level %d", __func__, debug_log_level);
        }
    }

    /* Initialize internal parameters defaults */

    /* Figure out the .rc filename */
    char * whoami = strrchr(g_whoami, '/');
    if (whoami == NULL) {
        /* In this case dsptop is in the PATH */
       whoami = g_whoami;
    } else {
        /* In this case dsptop is in the current directory 
         * So need to advance the pointer past the '/'
         */
        whoami++;
    }
    LOGMSG("%s: My name is %s", __func__, whoami);
    rc_filename[0] = '.';
    util_gen_filename(rc_filename + 1, filename_max_size - 1, whoami, "rc");
    options.rc_filename_p = rc_filename;

    /* Read the .dsptoprc file and evaluate the commands */
    parse_rc(&options);

    /* evaluate command line */ 
    parse_args(argc, argv, &options);

    LOGMSG("%s:Done parsing - Options:", __func__);
    LOGMSG("%s:  Test mode is : %d",__func__, options.test_mode);
    LOGMSG("%s:  Display update rate is : %f",__func__, options.delay_rate);
    LOGMSG("%s:  Moving average sample window rate is : %f",__func__, options.mva_window);    
    LOGMSG("%s:  Out filename: %s",__func__, options.out_filename_p); 
    LOGMSG("%s:  Maximum file size is : %d", __func__, options.max_filesize);
    LOGMSG("%s:  Sync enable is : %d", __func__, options.sync_enable);
    LOGMSG("%s:  Stop full is : %d", __func__, options.stop_full);
    LOGMSG("%s:  Process is : %d", __func__, options.process_until_exitmsg);
    LOGMSG("%s:  Out file format is : %d", __func__, options.format_type);
    LOGMSG("%s:  Plot resolution is : %f", __func__, options.plot_resolution);

    /* Handle any command line conflicts */
    if (options.logging_enable) {
        dsptop_info.dsptop_mode = LOGGING_MODE;
        if (options.out_filename_p == NULL) {
            options.out_filename_p = "stdout";
        }
    } else if (options.mva_window == 0) {
        dsptop_info.dsptop_mode = TOTAL_MODE;
    } 
    LOGMSG("%s:  Mode set to %s", __func__, io_get_modestr(dsptop_info.dsptop_mode));

    /* Quiet mode only valid if logging */
    if (!options.logging_enable && options.quiet_mode) {
       options.quiet_mode = false;
    } 
    LOGMSG("%s:  Quiet mode is %d", __func__, options.quiet_mode);

    /* Note - If test mode TEST_MODE_TIME_FAST or TEST_MODE_TIME_SLOW then
     * mva message generation in signal handler.
     */
    g_mva_test_mode = (test_mode_t)options.test_mode;

    /* Setup the iterations counter */
    uint32_t iterations_cnt = options.num_iterations;
    LOGMSG("%s:  Iterations count is %d", __func__, iterations_cnt);


    /* Get the device characteristics */
    dsptop_info.dsp_p = get_dspinfo();
    LOGMSG("%s:Device is %s, number of DSP is %d", __func__, dsptop_info.dsp_p->name, dsptop_info.dsp_p->num_dsps);

    /* Get space for run and idle time usage for each dsp */
    dsp_usage_info.run_tv_p = malloc(sizeof(struct timeval) * dsptop_info.dsp_p->num_dsps);
    dsp_usage_info.idle_tv_p = malloc(sizeof(struct timeval) * dsptop_info.dsp_p->num_dsps);
    if ((dsp_usage_info.run_tv_p == NULL) || (dsp_usage_info.idle_tv_p == NULL)) { 
        err_handler(ERR_TYPE_SYSTEM, ERR_MEM_ALLOC, NULL);
        /* Dummy return to keep code checker tools from declaring an issue,
         * err_handler will call exit.
         */
        return -1;
    }
    
    /* Initialize the signal handler */
    signal_handler_init();

    /* Initialize the display update rate */
    double display_rate_integral;
    display_rate_tv.tv_usec = (long)(modf(options.delay_rate, &display_rate_integral) * 1e6);
    display_rate_tv.tv_sec = (long)display_rate_integral;

    /* Clear the usage values */
    clr_usage_values(&dsptop_info);

    /* If not quiet mode then initialize the display */
    if (!options.quiet_mode) {
        io_init();
        io_update_screen(&dsptop_info, &dsp_usage_info);
    }

    /* If device does not have K2 Smart Reflex Sub System then don't try to initialize temperature support */
    if (dsptop_info.dsp_p->k2_srss) {
        temp_init(&dsp_usage_info.temp, &dsp_usage_info.temp_max);
    }
    plot_init(&options, dsptop_info.dsp_p);
    ulm_init(&options, dsptop_info.dsp_p, &dsp_usage_info);
    ulm_config_etb_recording();

    /* If in test mode TEST_MODE_SMALL_DATASET or TEST_MODE_LARGE_DATASET generate data */
    switch (options.test_mode) {
        case TEST_MODE_DISABLED:
            LOGMSG("%s: Test mode disabled", __func__);
            break;
        case TEST_MODE_SMALL_DATASET:
            LOGMSG("%s: Test mode enabled - small data set", __func__);
            ulm_generate_testdata();
            break;
        case TEST_MODE_LARGE_DATASET:
            LOGMSG("%s: Test mode enabled - large data set", __func__);
            ulm_generate_testlarge();
            break;
        /* For case 3 and 4 data generation handled by the signal handler */
        case TEST_MODE_TIME_FAST:
            LOGMSG("%s: Test mode enabled - timmed data set, fast", __func__);
            break;
        case TEST_MODE_TIME_SLOW:
            LOGMSG("%s: Test mode enabled - timmed data set, slow", __func__);
            break;
        default:
            LOGMSG("%s: Test mode is %d (invalid)", __func__, options.test_mode);
            break;
    }

    /* If sync enabled, time to write pid to fifo */
    if (options.sync_enable) {
        signal_handler_block();

        char * whoami = strrchr(g_whoami, '/');
        if (whoami == NULL) {
            /* In this case dsptop is in the PATH */
           whoami = g_whoami;
        } else {
            /* In this case dsptop is in the current directory 
             * So need to advance the pointer past the '/'
             */
            whoami++;
        }
        fifo_filename[0] = '.';
        util_gen_filename(fifo_filename + 1, filename_max_size - 1, whoami, "_fifo");

        LOGMSG("%s:fifo filename is %s", __func__, fifo_filename);
       
        util_pipe_open(fifo_filename);

        const size_t pid_size = 8; 
        char pid_string[pid_size];

        pid_t mypid = getpid();
        snprintf(pid_string, pid_size, "%d\n", mypid);
        LOGMSG("%s:Write to pipe pid: %s", __func__, pid_string);

        util_pipe_write(pid_string, strlen(pid_string) + 1);
        util_pipe_close();
        signal_handler_unblock();  

    }

    /*  Initialize reset_delta flag for processing loop */
    bool reset_delta = false;

    /* Initialize delta elapsed time for processing loop */
    struct timeval delta_tv = {
        .tv_sec = 0,
        .tv_usec = 0
    };

    /* Enable 5000 usec timer. 
     * This should always be the last step before entering the processing loop.
     * Note: Do not call LOGMSG functions while signals are not blocked. LOGMSG
     * calls non-reenterent functions, and LOGMSG is called from the signal handler.  
     */
    set_itimer();

    /* Processing Loop*/
    do {
        /* Update the screen */
        signal_handler_block();

        get_elapsedtime(&dsptop_info.elapsed_tv, &delta_tv, reset_delta);

        /* Note if logging this function returns 0 for gap*/
        ulm_get_gaptime(&dsptop_info.gap_tv);

        LOGMSG("%s:delta time is %d.%06d", __func__, delta_tv.tv_sec, delta_tv.tv_usec);
        LOGMSG("%s:gap time is %d.%06d", __func__, dsptop_info.gap_tv.tv_sec, dsptop_info.gap_tv.tv_usec);
        LOGMSG("%s:resize_window_event is %d",__func__, global_state.resize_window_event);
    
        if (reset_delta == true) {
            reset_delta = false;
        }

        /* Update the display if the delta time is > the display refresh rate
         * OR the screen needs resized.
         */
        if ( ((delta_tv.tv_sec >= display_rate_tv.tv_sec)
           && (delta_tv.tv_usec >= display_rate_tv.tv_usec))
           || global_state.resize_window_event) {

            if (global_state.resize_window_event) {
                io_resize_screen();
                global_state.resize_window_event = false;
            }

            io_update_screen(&dsptop_info, &dsp_usage_info);

            reset_delta = true;
            
            /* If num_iterations is non-zero then decrement the
             * iterations counter and if zero exit.
             */
            if (options.num_iterations != 0) {
                if (--iterations_cnt == 0) {
                    term(dsptop_info.dsp_p);
                }

                LOGMSG("%s:iterations count is %d", __func__, iterations_cnt);
            }

        }

        signal_handler_unblock();

        /* If quiet mode then interactive commands are disabled */
        if (options.quiet_mode) {
            usleep(wait_time_quiet_usec);
            /* Skip io_wait() call */
            continue;
        }

        /* Wait for any user input. If no input in .1 second then
         * simply loop back to check if a screen update is necessary
         */
        if (io_wait()) {

            signal_handler_block();

            LOGMSG("%s:Command char available", __func__);
            /* Get a command, if invalid keep trying */
            interactive_cmd_t cmd;
            do {
                cmd = io_get_cmd(&dsptop_info);

                if ((cmd == DSPTOP_INVALID) || (cmd == DSPTOP_INVALID_LOGGING)) {
                    io_put_cmdpromt(cmd, NULL);
                }

            } while(cmd == DSPTOP_INVALID);

            if (cmd != DSPTOP_NO_COMMAND) {
                eval_cmd(cmd);
            }

            signal_handler_unblock();
        } /* End of "if (iowait()) {" */  

    } while(1); 
 
}


/******************************************************************************
 * term()
 *      Terminate the utility
 *****************************************************************************/
void term() 
{
    LOGFUNC();

    char * msg = NULL;
    int etb_avaiable_words = 0;
    int etb_percent_full;
    bool is_etb_enabled;
    bool is_etb_wrapped;
    bool is_logging;
    bool stop_trigger;
    bool log_file_written = false;

    if (!options.quiet_mode) {
        io_term();
    }

    if(options.logging_enable) {
        etb_avaiable_words = ulm_stop_etb_recording();

        ulm_get_etb_state(&etb_percent_full, &is_etb_enabled, &is_etb_wrapped, &is_logging, &stop_trigger);
        if (is_etb_enabled == true) {
            err_handler(ERR_TYPE_LOCAL, ERR_ETB_ENABLED_STATE, NULL);
            /* Dummy return to keep code checker tools from declaring an issue,
             * err_handler will call exit.
             */
            return;    
        } 

        if (etb_avaiable_words != 0) {
            log_file_written = ulm_decode_to_logfile(options.out_filename_p, dsptop_info.dsp_p);

            if (log_file_written == true) {
                if (stop_trigger) {
                    msg = "Trace buffer stopped on full";
                } else if (is_etb_wrapped) {
                    msg = "Trace buffer wrapped";
                } else {
                    msg = "Trace buffer not wrapped";
                }

                fprintf(g_stdout, "Terminating logging mode, %d ULM message bytes collected, Trace Buffer %d%% full: %s\n",
                        etb_avaiable_words * 4, etb_percent_full, msg);
               
            }
        }

        if (log_file_written == false) {
            fprintf(g_stdout, "Terminating logging mode with no ULM messages collected\n");
        }           
 
    }


    if (!options.logging_enable) {

        LOGMSG3("%s", (dsptop_info.dsptop_mode == TOTAL_MODE) ? "Total Mode" : "Moving Average Mode");

        ulm_log_lastusage();

        LOGMSG3("External Mem Code/Data %d KB Total, %3.1f%% Used", 
                dsp_usage_info.extern_cd.num_1kb_blocks, 100.0 - dsp_usage_info.extern_cd.min_percent_free);
        
        LOGMSG3("External Mem Data %d KB Total, %3.1f%% Used", 
                dsp_usage_info.extern_d.num_1kb_blocks, 100.0 - dsp_usage_info.extern_d.min_percent_free);
         
        LOGMSG3("Internal Mem Data %d KB Total, %3.1f%% Used", 
                dsp_usage_info.intern_d.num_1kb_blocks, 100.0 - dsp_usage_info.intern_d.min_percent_free);

        LOGMSG3("ARM Freq %d MHz, DSP Freq %d Mhz", util_get_freq(ARM_CORE_CLK)/(long)1e6, util_get_freq(DSP_CORE_CLK)/(long)1e6);

        if (dsp_usage_info.temp > -127) {
            LOGMSG3("Current Device Temperature %d C, Max Device Temperature %d C", dsp_usage_info.temp, dsp_usage_info.temp_max);             
        } else {
            LOGMSG3("Device temperature sensor not calibrated");
            
        }

        LOGMSG3("Total ulm messages collected %d", g_total_ulm_msgcnt);

    }

    plot_postprocess(); 
    plot_launch();
    plot_close();

    temp_term();

    ulm_close();    
    exit(0);
} 

/***************************************************************************** 
 *  Private functions
 *****************************************************************************/

/***************************************************************************** 
 * eval_cmd 
 *
 * - Evaluate an interactive command
 *
 *****************************************************************************/
static void eval_cmd(interactive_cmd_t cmd) 
{
    LOGFUNC();

    /* Evaluate the command */
    switch (cmd) {
        case DSPTOP_INVALID:            /* Note: eliminate a compiler warning */
        case DSPTOP_INVALID_LOGGING:    /* Note: eliminate a compiler warning */
        case DSPTOP_HELP:
        {
            io_help(&dsptop_info, &options);
            /* Wait for a char to switch display back */
            io_put_cmdpromt(cmd, NULL);
            io_getch();
            break;
        }
        case DSPTOP_DISPLAY_UPDATE:
        {
            io_put_cmdpromt(cmd, &options.delay_rate);
            /* If there is an error the original value is returned */                    
            double new_delay_rate = io_get_doublevalue(options.delay_rate);
            
            /* Only update if non-zero number returned */
            if (new_delay_rate != 0) {
                /* Adjust new_delay_rate to >.5 && < 900 seconds. 
                 * Note: adjust_delay() is a macro defined in command.h 
                 */
                adjust_delay(new_delay_rate);
                options.delay_rate = new_delay_rate;

                double display_rate_sec;
                display_rate_tv.tv_usec = (long)(modf(options.delay_rate, &display_rate_sec) * 1e6);
                display_rate_tv.tv_sec = (long)display_rate_sec;           
            } else {
                io_invalid();
                io_restore_prompt();
            }
            break;
        }
        case DSPTOP_MOVING_AVERAGE:
        {
            io_put_cmdpromt(cmd, &options.mva_window);
            /* If there is an error the original value is returned */                    
            double new_sample_window = io_get_doublevalue(options.mva_window);
            
            if (new_sample_window != options.mva_window) {
                /* Adjust new_sample_window if != 0 to >.5 && < 900 seconds. 
                 * Note: adjust_mva_window() is a macro defined in command.h 
                 */
                adjust_mva_window(new_sample_window); 
                options.mva_window = new_sample_window;

                if (new_sample_window != 0) {
                    dsptop_info.dsptop_mode = MOVING_AVERAGE_MODE;
                    LOGMSG("%s:Switch to Moving Average Mode", __func__);
                } else {
                    dsptop_info.dsptop_mode = TOTAL_MODE;
                    LOGMSG("%s:Switch to Total Mode", __func__);
                }

                ulm_stop_etb_recording();
                /* If plotting ulm_init() call will clear flags used to restart plotting */
                plot_init(&options, dsptop_info.dsp_p);
                ulm_init(&options, dsptop_info.dsp_p, &dsp_usage_info);
                ulm_config_etb_recording();
            }
            break;
        }
        case DSPTOP_WRITE_CONFIG:
        {
            write_rc(&options);
            io_put_cmdpromt(cmd, NULL);
            break;
        }
        case DSPTOP_PLOT_RESOLUTION_UPDATE:
        {
            io_put_cmdpromt(cmd, &options.plot_resolution);
            /* If there is an error the original value is returned */                    
            double new_plot_resolution = io_get_doublevalue(options.plot_resolution);
            
            /* Only update if non-zero number returned */
            if (new_plot_resolution != 0) {
                /* Adjust new_plot_resolution to >.001 && < 1 seconds. 
                 * Note: adjust_plot_resolution() is a macro defined in command.h 
                 */
                adjust_plot_resolution(new_plot_resolution);
                options.plot_resolution = new_plot_resolution;

            } else {
                io_invalid();
                io_restore_prompt();
                break;
            }
            /* Fall through to clear */
            cmd = DSPTOP_CLEAR;
        }    
        case DSPTOP_CLEAR:
        {
            clr_usage_values(&dsptop_info);
            ulm_stop_etb_recording();
            plot_init(&options, dsptop_info.dsp_p);
            ulm_init(&options, dsptop_info.dsp_p, &dsp_usage_info);
            ulm_config_etb_recording();
            io_put_cmdpromt(cmd, NULL);
            break;
        }
        case DSPTOP_QUIT:
        {
            term(); 
            break;
        }
        case DSPTOP_NO_COMMAND: 
            break;
    }
}

/******************************************************************************
 * clr_usage_values()
 *      
 * Clear values used by the display
 *****************************************************************************/
void clr_usage_values(dsptop_info_t * dsptop_info_p)
{
    struct timeval clear_tv = {
        .tv_sec = 0,
        .tv_usec = 0,
    };

    int num_dsps = 1;

    if (dsptop_info_p != NULL) {
        num_dsps = dsptop_info_p->dsp_p->num_dsps;
    }

    /* Clear the elapsed time */
    initial_tv_p = NULL;
    /* Clear the gap time */
    ulm_clr_gaptime();
    
    /* Initialize the screen values to all zeros */
    for ( int i = 0; i < num_dsps; i++) {
        dsp_usage_info.run_tv_p[i] = clear_tv;
        dsp_usage_info.idle_tv_p[i] = clear_tv;

    }
    dsp_usage_info.extern_cd.num_1kb_blocks = 0;
    dsp_usage_info.extern_cd.percent_free = 100.0;
    dsp_usage_info.extern_cd.min_percent_free = 100;
    dsp_usage_info.extern_d.num_1kb_blocks = 0;
    dsp_usage_info.extern_d.percent_free = 100.0;
    dsp_usage_info.extern_d.min_percent_free = 100;
    dsp_usage_info.intern_d.num_1kb_blocks = 0;
    dsp_usage_info.intern_d.percent_free = 100.0;
    dsp_usage_info.intern_d.min_percent_free = 100;
    dsp_usage_info.temp = -128;
}
/******************************************************************************
 * set_itimer()
 *      
 * Setup interval timer
 *****************************************************************************/
static void set_itimer()
{
    int err;
    /* Setup the interval timer */

    struct itimerval delay;
    struct timeval intraval;

    LOGMSG("%s:setting ITIMER_REAL for %d micro-second resolution", __func__, wait_time_timer_usec);

    intraval.tv_sec = 0;
    intraval.tv_usec = wait_time_timer_usec;

    delay.it_interval = intraval;
    delay.it_value = intraval;
    
    err = setitimer(ITIMER_REAL, &delay, NULL);
    if (err == -1) {
        err_handler(ERR_TYPE_SYSTEM, ERR_DEBUG, NULL);
        /* Dummy return to keep code checker tools from declaring an issue,
         * err_handler will call exit.
         */
        return;
    }

    /* Note the signal handler for setitimer has already been initialized
     * by the call to signal_handler_init() from main.
     */

}

/******************************************************************************
 * get_dspinfo()
 *      Provides number of dsps in the device
 *****************************************************************************/

static dsp_type_t * get_dspinfo()
{
#ifdef SUDO_BUILD
    return &dsp_type_table[0];
#else
    char * device_name = util_get_device();

    /* Search the table for device name */
    /* Note: num_elements is -1 so we don't compare
     * against "unknown" at end of table.
     */
    int num_elements = sizeof(dsp_type_table)/sizeof(dsp_type_t) - 1;
    int i;

    for(i = 0; i < num_elements; i++) {
        if (!strncmp(device_name, dsp_type_table[i].name, strlen(device_name))) {
            break;
        }
    }

    return &dsp_type_table[i];
#endif
}

/******************************************************************************
 * get_elapsedtime()
 *      Provides elapsed timestamp string in HH:MM:SS:MS 
 *****************************************************************************/

static void get_elapsedtime(struct timeval * elapsed_tv, struct timeval * delta_tv, bool reset_delta)
{
    struct timeval current_tv;

    /* Check if we need the initial value */
    if (initial_tv_p == NULL) {
        gettimeofday(&initial_tv, NULL);
        initial_tv_p = &initial_tv;

        elapsed_tv->tv_sec = 0;
        elapsed_tv->tv_usec = 0;

        *delta_tv = *elapsed_tv;

        previous_tv = initial_tv;    

    } else {
        int adj_sec = 0;
        int adj_usec = 0;        

        gettimeofday(&current_tv, NULL);
        /* Calculate elapsed time from the start */
        if (initial_tv.tv_usec > current_tv.tv_usec) {
            /* Need to borrow a second for usec */
            adj_sec = -1;
            adj_usec = 1000000;
        }
        
        elapsed_tv->tv_usec = current_tv.tv_usec + adj_usec - initial_tv.tv_usec;
        elapsed_tv->tv_sec = current_tv.tv_sec + adj_sec - initial_tv.tv_sec;

        /* Calculate the delta time from the previous */
         if (previous_tv.tv_usec > current_tv.tv_usec) {
            /* Need to borrow a second for usec */
            adj_sec = -1;
            adj_usec = 1000000;
        } else {
            adj_sec = 0;
            adj_usec = 0;
        }  
        
        if (reset_delta) {
            delta_tv->tv_usec = current_tv.tv_usec + adj_usec - previous_tv.tv_usec;
            delta_tv->tv_sec = current_tv.tv_sec + adj_sec - previous_tv.tv_sec;
        } else {
            delta_tv->tv_usec += current_tv.tv_usec + adj_usec - previous_tv.tv_usec;
            delta_tv->tv_sec += current_tv.tv_sec + adj_sec - previous_tv.tv_sec;
            
            if (delta_tv->tv_usec >= 1000000) {
                delta_tv->tv_usec -= 1000000;
                delta_tv->tv_sec++;
            }
        }

        previous_tv = current_tv;

    }

}

