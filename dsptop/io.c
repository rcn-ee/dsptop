/*
 * io.c
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
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>

#include "dsptop.h"
#include "error_handler.h"
#include "debug_log.h"
#include "io.h"
#include "utility.h"
#include "signal_handler.h"
#include "ulm_handler.h"

#include "ncurses.h"

/***************************************************************************** 
 *  Internal Function Prototypes 
 *  - See Private Function section below for implementations )
 *****************************************************************************/

/*****************************************************************************  
 *  Static Definitions and Initialization
 *****************************************************************************/
static const char * dsptop_mode_name_table[] = {
    [MOVING_AVERAGE_MODE] = "Moving Average Mode",
    [TOTAL_MODE] = "Total Mode",
    [LOGGING_MODE] = "Logging Mode"
};


static const char * cmd_prompt_table[] = {
    [DSPTOP_INVALID] = "Enter a valid command [?cdhsqw]",
    [DSPTOP_INVALID_LOGGING] = "Enter a valid command for logging[?hqw]",
    [DSPTOP_HELP] = "Enter any char to exit help",
    [DSPTOP_DISPLAY_UPDATE] = "Change delay from %2.3f to: ",
    [DSPTOP_MOVING_AVERAGE] = "Change moving average from %2.3f to: ",
    [DSPTOP_PLOT_RESOLUTION_UPDATE] = "Change plot resolution from %2.3f to:",
    [DSPTOP_WRITE_CONFIG] = "Current configuration written to .dsptoprc", 
    [DSPTOP_CLEAR] = "Clear current values and start over",
    [DSPTOP_QUIT] = NULL,
    [DSPTOP_NO_COMMAND] = NULL
};

static const char * cmd_enum_table[] = {
    [DSPTOP_INVALID] = "DSPTOP_INVALID",
    [DSPTOP_INVALID_LOGGING] = "DSPTOP_INVALID_LOGGING",
    [DSPTOP_HELP] = "DSPTOP_HELP",
    [DSPTOP_DISPLAY_UPDATE] = "DSPTOP_DISPLAY_UPDATE",
    [DSPTOP_MOVING_AVERAGE] = "DSPTOP_MOVING_AVERAGE",
    [DSPTOP_PLOT_RESOLUTION_UPDATE] ="DSPTOP_PLOT_RESOLUTION_UPDATE",
    [DSPTOP_WRITE_CONFIG] = "DSPTOP_WRITE_CONFIG", 
    [DSPTOP_CLEAR] = "DSPTOP_CLEAR",
    [DSPTOP_QUIT] = "DSPTOP_QUIT",
    [DSPTOP_NO_COMMAND] = "DSPTOP_NO_COMMAND"
};


#define INPUT_BUF_SIZE 16
static char input_buf[INPUT_BUF_SIZE];

/* Display parameters */
static const int summary_title_row = 0;
static const int summary_mode_row = 1;
static const int summary_usage_row = 2;

static const int summary_mem_ex_cd = 3;
static const int summary_mem_ex_d = 4;
static const int summary_mem_in_d = 5;
static const int summary_device_op = 6;

static const int interactive_blank_row = 7;
static const int interactive_row = 8;
static const int help_row = 9;
static const int dsp_usage_title_row = 9;
static const int dsp_usage_start_row = 10;

static const int interactive_row_logging_adjust = 5;

static int num_rows_adjust = 0;

/* Signal data queue */
static char io_queue_data = '\0';

static bool io_init_complete = false;

/***************************************************************************** 
 * Public Functions
 *
 *****************************************************************************/

/***************************************************************************** 
 *  io_get_modestr()
 *
 *  return a char * to the mode string
 *
 *****************************************************************************/ 
const char * io_get_modestr(dsptop_mode_t dsptop_mode)
{
    return dsptop_mode_name_table[dsptop_mode];  
}

/***************************************************************************** 
 *  io_init()
 *
 *  Initialize the curses library
 *
 *****************************************************************************/ 
void io_init()
{
    LOGFUNC();

    /* Initialize curses 		  */
    initscr();
    noecho();
    cbreak();

    io_init_complete = true;  
}

/***************************************************************************** 
 *  io_term()
 *
 *  Terminate the curses library
 *
 *****************************************************************************/
void io_term()
{
    LOGFUNC();

    if (io_init_complete == false) {
        return;
    }

    /* Terminate curses 		  */
    endwin();

    io_init_complete = false;

    /* Note: must printf the current screen if it's ever required to retain 
     * dsptop values after the window is terminated. This is "top" utility behavior.
     */
  
}

/***************************************************************************** 
 *  io_update_screen()
 *
 *  Refresh the screen
 *
 *  Note: dsp_usage_info_p is a pointer to an array of dsp_usage_info[dsp_cnt]
 *****************************************************************************/ 
void io_update_screen(dsptop_info_t * dsptop_info_p, dsp_usage_info_t * dsp_usage_info_p)
{
    LOGFUNC();

    erase();
    refresh();

    /* Enough space for usecs, although only display msec */
    const size_t time_buf_max = 17;  
    char elapsed_time_buf[time_buf_max];
    char run_time_buf[time_buf_max];
    char idle_time_buf[time_buf_max];

    size_t used = util_get_timestring(elapsed_time_buf, time_buf_max, &dsptop_info_p->elapsed_tv, TIMESTRING_SECOND);
    if (used > time_buf_max) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    mvprintw(summary_title_row, 0, "%s - Elapsed Time %s", g_whoami, elapsed_time_buf);
    clrtoeol();
    mvprintw(summary_mode_row, 0, "%s", dsptop_mode_name_table[dsptop_info_p->dsptop_mode]);
    clrtoeol();

    if (dsptop_info_p->dsptop_mode != LOGGING_MODE) {

        /* Determine the number of memory usage rows not being displayed */
        num_rows_adjust = (dsp_usage_info_p->extern_cd.num_1kb_blocks == 0) ? 1 : 0;
        int num_ext_d_adj = num_rows_adjust;
        num_rows_adjust += (dsp_usage_info_p->extern_d.num_1kb_blocks == 0) ? 1 : 0;
        int num_int_d_adj = num_rows_adjust;
        num_rows_adjust += (dsp_usage_info_p->intern_d.num_1kb_blocks == 0) ? 1 : 0;

        /* Display memory and temperature summary */
        if (dsp_usage_info_p->extern_cd.num_1kb_blocks != 0) {
            mvprintw(summary_mem_ex_cd, 0, "External Mem Code/Data %d KB Total, %3.1f%% Used", 
                    dsp_usage_info_p->extern_cd.num_1kb_blocks, 100.0 - dsp_usage_info_p->extern_cd.percent_free);
            clrtoeol();
        }
        if (dsp_usage_info_p->extern_d.num_1kb_blocks != 0) {
            mvprintw(summary_mem_ex_d - num_ext_d_adj, 0, "External Mem Data %d KB Total, %3.1f%% Used", 
                    dsp_usage_info_p->extern_d.num_1kb_blocks, 100.0 - dsp_usage_info_p->extern_d.percent_free);
            clrtoeol();
        }
        if (dsp_usage_info_p->intern_d.num_1kb_blocks != 0) {
            mvprintw(summary_mem_in_d - num_int_d_adj, 0, "Internal Mem Data %d KB Total, %3.1f%% Used", 
                    dsp_usage_info_p->intern_d.num_1kb_blocks, 100.0 - dsp_usage_info_p->intern_d.percent_free);
            clrtoeol();
        }

        /* Display temperature information, if temperature not dispalyed adjust number of rows */
        if (dsp_usage_info_p->temp > -127) {
            mvprintw(summary_device_op - num_rows_adjust, 0, "Current Device Temperature %d C, Max Device Temperature %d C", dsp_usage_info_p->temp, dsp_usage_info_p->temp_max);
            clrtoeol();
        } else {
            num_rows_adjust++;
        }

        wmove(stdscr, interactive_blank_row - num_rows_adjust, 0);
        clrtoeol();

        /* determine if time should be displayed in milliseconds 
         * or microseconds resolution.
         */
        timestring_resolution_t time_res = TIMESTRING_MILLISECOND;
        for (int i = 0; i < dsptop_info_p->dsp_p->num_dsps; i++) {

            double runtime = dsp_usage_info_p->run_tv_p[i].tv_sec + (double)dsp_usage_info_p->run_tv_p[i].tv_usec/1e6;
            double idletime = dsp_usage_info_p->idle_tv_p[i].tv_sec + (double)dsp_usage_info_p->idle_tv_p[i].tv_usec/1e6;

            if (((runtime > 0) && (runtime < .001)) || ((idletime > 0) && (idletime < .001))){
                time_res = TIMESTRING_MICROSECOND;
                break;
            }   

        }

        /* Display DSP usage */
        int numspace = (time_res == TIMESTRING_MILLISECOND) ? 1 : 3;
        attron(A_REVERSE);
        mvprintw(dsp_usage_title_row - num_rows_adjust, 0, "DSP    %USAGE  %*sRUN TIME+    %*sIDLE TIME+%*s", numspace, "", numspace, "", numspace, "");
        attroff(A_REVERSE);
        clrtoeol();

        double accum_usage = 0;

        for (int i = 0; i < dsptop_info_p->dsp_p->num_dsps; i++){

            double usage;
            
            double runtime = dsp_usage_info_p->run_tv_p[i].tv_sec;
            runtime += (double)dsp_usage_info_p->run_tv_p[i].tv_usec/1e6;
            
            double idletime = dsp_usage_info_p->idle_tv_p[i].tv_sec;
            idletime += (double)dsp_usage_info_p->idle_tv_p[i].tv_usec/1e6;

            if (runtime == 0) {
                usage = 0;
            }else {
                usage = (runtime/(runtime + idletime))*100.0;
            }

            used = util_get_timestring(run_time_buf, time_buf_max, &(dsp_usage_info_p->run_tv_p[i]), time_res);
            if (used > time_buf_max) {
                err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
            }            

            used = util_get_timestring(idle_time_buf, time_buf_max, &(dsp_usage_info_p->idle_tv_p[i]), time_res);
            if (used > time_buf_max) {
                err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
            }    

            mvprintw(dsp_usage_start_row - num_rows_adjust + i, 0, "DSP_%d  %6.2f  %s  %s",
                     i, usage, run_time_buf, idle_time_buf);       

            LOGMSG("%s: DSP_%d  usage=%6.2f  runtime=%s  idletime=%s", __func__, i, usage, run_time_buf, idle_time_buf);

            accum_usage += usage;

        } 

        double elapsedtime = dsptop_info_p->elapsed_tv.tv_sec;
        elapsedtime += (double)dsptop_info_p->elapsed_tv.tv_usec/1e6;    

        double gaptime = dsptop_info_p->gap_tv.tv_sec;
        gaptime += (double)dsptop_info_p->gap_tv.tv_usec/1e6;

        double accuracy;

        if (gaptime == 0) {
            accuracy = 100.0;
        }else {
            accuracy = 100.0 -((gaptime/elapsedtime)*100.0);
        }

        /* Check for divide by zero so NAN does not get displayed */
        double average_dsp_usage = 0;
        if ((accum_usage) != 0) {
            average_dsp_usage = accum_usage/(double)dsptop_info_p->dsp_p->num_dsps;
        } 

        mvprintw(summary_usage_row, 0, "Average DSP Usage: %5.2f%% Accuracy %5.2f%%", average_dsp_usage, accuracy);

    } else {

        int etb_percent_full;
        bool is_etb_enabled;
        bool is_etb_wrapped;
        bool is_logging;
        bool stop_trigger;
        int mode_len = strlen(dsptop_mode_name_table[dsptop_info_p->dsptop_mode]);
        
        ulm_get_etb_state(&etb_percent_full, &is_etb_enabled, &is_etb_wrapped, &is_logging, &stop_trigger);
        
        mvprintw(summary_mode_row, mode_len + 4, "Trace Buffer is %d%% full", etb_percent_full);

        /* For LOGGING_MODE back up the interactive row by 1 */
        num_rows_adjust = interactive_row_logging_adjust;
    }
    /* Move the cursor to the interactive line
     * Note: this must be the last item in the screen update list.
     */
    mvprintw(interactive_row - num_rows_adjust, 0, ">");
    clrtoeol();
	refresh();			/* Print it on to the real screen */

}

/***************************************************************************** 
 *  io_resize_screen()
 *
 *  Resize the screen
 *
 *****************************************************************************/ 
void io_resize_screen()
{
    LOGFUNC();

    endwin();
    refresh();
    clear();

}

/***************************************************************************** 
 *  io_help()
 *
 *  Refresh the screen with help
 *
 * 
 *****************************************************************************/ 
void io_help(dsptop_info_t * dsptop_info_p, options_t * options_p)
{
    LOGFUNC();

    int start_row = help_row - num_rows_adjust;
    int first_tab = 4;

    move(start_row, 0);
    clrtobot();

    mvprintw(start_row,0, "Interactive Commands for %s:", dsptop_mode_name_table[dsptop_info_p->dsptop_mode]);
    mvprintw(++start_row, first_tab, "?,h - Help");
    if (dsptop_info_p->dsptop_mode != LOGGING_MODE) {
        mvprintw(++start_row, first_tab, "d   - Change the display update interval time");
        mvprintw(++start_row, first_tab, "s   - Change the moving average sample window time");
        mvprintw(++start_row, first_tab, "        Note: Set to 0 changes operation to Total Mode");
        mvprintw(++start_row, first_tab, "c   - Clear the current values and start over");
        if (options_p->out_filename_p != NULL) {
            mvprintw(++start_row, first_tab, "r   - Change plot resolution and Clear");  
        }
        mvprintw(++start_row, first_tab, "q   - Quit dsptop");
        mvprintw(++start_row, first_tab, "CNT-C - Quit dsptop (from terminal or script)");
    } else {
        mvprintw(++start_row, first_tab, "q   - Update the log file and quit dsptop");
        mvprintw(++start_row, first_tab, "CNT-C - Update the log file and quit dsptop");
        mvprintw(++start_row, first_tab, "          (from terminal or script)");
    }
        mvprintw(++start_row, first_tab, "w   - Write the current configuration to .dsptoprc");


    ++start_row;

    mvprintw(++start_row, 0, "Device is %s", dsptop_info_p->dsp_p->name);
    /* Only print ARM and DSP frequencies if K2 device, Don't know how to get for other devices. */
    if (dsptop_info_p->dsp_p->k2_srss) {
        mvprintw(++start_row, 0, "ARM Freq %d MHz, DSP Freq %d Mhz", 
                 util_get_freq(ARM_CORE_CLK)/(long)1e6, util_get_freq(DSP_CORE_CLK)/(long)1e6);
    }

    ++start_row;

    if (dsptop_info_p->dsptop_mode != LOGGING_MODE) {

        mvprintw(++start_row, 0, "Display Update Interval: %2.3f", options_p->delay_rate);
        if (options_p->mva_window != 0) {
            mvprintw(++start_row, 0, "Moving Average Sample Window: %2.3f", options_p->mva_window);
        } else {
            mvprintw(++start_row, 0, "Total Mode (infinite sample window)", options_p->mva_window);
        }
        if (options_p->process_until_exitmsg) {
             mvprintw(++start_row, 0, "Single DSP process run/idle time accumulation");
        } else {
             mvprintw(++start_row, 0, "Continuous DSP run/idle time accumulation");
        }

        if (options_p->out_filename_p != NULL) {
            mvprintw(++start_row, 0, "Usage plot output file is: %s", options_p->out_filename_p);
            mvprintw(++start_row, 0, "Usage plot output file format is: %s", get_option_argument('f', options_p->format_type));
            if (options_p->max_filesize == 0) {
                mvprintw(++start_row, 0, "Maximum output file size is: unlimited");
            } else {    
                mvprintw(++start_row, 0, "Maximum output file size is: %d bytes", options_p->max_filesize);
            }
            mvprintw(++start_row, 0, "Plot resolution in seconds is: %2.3f", options_p->plot_resolution);
        }        

    } else {
        mvprintw(++start_row, 0, "Log file is: %s", options_p->out_filename_p);
        if (options_p->max_filesize == 0) {
            mvprintw(++start_row, 0, "Max log file size is: infinite");
        } else {    
            mvprintw(++start_row, 0, "Max log file size is: %d bytes", options_p->max_filesize);
        }

        if (options_p->stop_full){
            mvprintw(++start_row, 0, "Buffer type is: Stop when full");
        } else {
            mvprintw(++start_row, 0, "Buffer type is: Circular");
        }
    }

    ++start_row;
    mvprintw(++start_row, 0, "Ask a question or report a problem at e2e.ti.com");
    mvprintw(++start_row, 0, "%s version is: %d.%d.%d", g_whoami, g_major_version, g_minor_version, g_patch_version);
    mvprintw(++start_row, 0, "Copyright (C) %d Texas Instruments, Inc.\n", g_copyright_year);    
}

/***************************************************************************** 
 *  io_wait()
 *
 *  Wait for input on stdin or tmeout every .1 seconds
 *
 *  Returns true if a char is available on stdin.
 *  Returns false on a timeout.
 * 
 *****************************************************************************/ 
bool io_wait()
{
    /* All functions called from io_wait() must be reenterent. So don't try to call
     * LOCFUNC or LOGMSG from this function (they are not reenterent).
     */ 

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 100000
    };

    fd_set read_fdset;
    int stdio_fd = fileno(stdin);
    int nfd_set;
    int retry = 10;

    FD_ZERO(&read_fdset);
    FD_SET(stdio_fd, &read_fdset);

    do {
        nfd_set = select(stdio_fd+1, &read_fdset, NULL, NULL, &tv);

        if (nfd_set >= 0) {
            break;
        }

        /* Most likely this was caused by SIGALRM */
        if (nfd_set == -1 && errno == EINTR) {
            nfd_set = 0;
        } else {
            break;
        }

        if (tv.tv_usec < 10000) {
            break;
        }

    } while (retry-- > 0);

    if (nfd_set < 0) {
        err_handler(ERR_TYPE_SYSTEM, ERR_IO, NULL);
    }

    /* If there is a command queued from the signal handler
     * then increment nfd_set.
     */
    if ((nfd_set == 0) && (io_queue_data != '\0')) {
        nfd_set++;
    }

    return nfd_set;

}

/***************************************************************************** 
 *  io_get_cmd()
 *
 *  Get a char from stdio and return the corresponding command.
 *
 *  Warning: Since the call to getch is blocking, never call this unless you
 *           are already sure a char is available. 
 * 
 *****************************************************************************/ 
interactive_cmd_t io_get_cmd(dsptop_info_t * dsptop_info_p)
{
    int ret_cmd;
    bool ch_available;
    char ch;

    LOGFUNC();
    
    /* Get queued input or wait for user input */
    ch = io_queue_getch(&ch_available);
    if (!ch_available) {
        ch = getch();
    }

    switch (ch) {
        case 'h':
        case '?':
            ret_cmd =  DSPTOP_HELP;
            break;
        case 'd':
            ret_cmd = (dsptop_info_p->dsptop_mode == LOGGING_MODE) ? DSPTOP_INVALID_LOGGING : DSPTOP_DISPLAY_UPDATE;
            break;
        case 'r':
            ret_cmd = DSPTOP_PLOT_RESOLUTION_UPDATE;
            break;
        case 's':
            ret_cmd = (dsptop_info_p->dsptop_mode == LOGGING_MODE) ? DSPTOP_INVALID_LOGGING : DSPTOP_MOVING_AVERAGE;
            break;
        case 'w':
            ret_cmd = DSPTOP_WRITE_CONFIG;
            break;
        case 'c':
            ret_cmd = (dsptop_info_p->dsptop_mode == LOGGING_MODE) ? DSPTOP_INVALID_LOGGING : DSPTOP_CLEAR;
            break;
        case 'q':
            ret_cmd = DSPTOP_QUIT;
            break;
        case 255:                        /* Most likely caused by CNTL char */
        case 27:                         /* ESC*/
        case 32:                         /* Space*/
            ret_cmd = DSPTOP_NO_COMMAND; 
            break;
        default:
            ret_cmd = (dsptop_info_p->dsptop_mode == LOGGING_MODE) ? DSPTOP_INVALID_LOGGING : DSPTOP_INVALID;
    }
    
    LOGMSG("%s: Returning command %s - ch is %d", __func__, cmd_enum_table[ret_cmd], ch);
    return ret_cmd;
}

/***************************************************************************** 
 *  io_queue_ch()
 *
 *  Queue a character, returns true if ch queued, false if no room in queue.
 * 
 *****************************************************************************/ 
bool io_queue_putch(char ch)
{
    LOGFUNC(); 

    if (io_queue_data == '\0') {
        io_queue_data = ch;
        return true;
    } else {
        return false;
    }

}

/***************************************************************************** 
 *  io_queue_getch()
 *
 *  Get a character from the queue.
 * 
 *****************************************************************************/ 

char io_queue_getch(bool * ch_available)
{
    LOGFUNC();
    char return_ch ='\0'; 
    
    /* If signals are not blocked a signal function
     * could call io_queue_putch() while io_queue_getch
     * is being executed.
     */

    if (io_queue_data != 0) {
        *ch_available = true;
        return_ch = io_queue_data;
        LOGMSG ("%s: returning %c", __func__, return_ch);
        io_queue_data = '\0';
    } else {
        *ch_available = false;        
    }

    return return_ch;
}

/***************************************************************************** 
 *  io_getch()
 *
 *  Get a char from stdio and return the character.
 *
 *  Warning: Since the call to getch is blocking, never call this unless you
 *           are already sure a char is available. 
 * 
 *****************************************************************************/ 
char io_getch()
{
    LOGFUNC();
    char ch;
    
    do {
        ch = getch();

        if ((ch == (char)ERR) && (errno == EINTR)) {
            continue;
        } else {
            break;
        }

    } while (1);
    
    LOGMSG("%s: Returning %c", __func__, ch);

    /* Wait for user input */
    return(ch);
}
/***************************************************************************** 
 *  io_put_cmdpromt()
 *
 *  Put a command prompt message on the interactive display row.
 * 
 *****************************************************************************/ 
void io_put_cmdpromt(interactive_cmd_t cmd, double * value_p)
{
    LOGFUNC();

    if ( value_p == NULL ) {    
        mvprintw(interactive_row - num_rows_adjust, 1, "%s", cmd_prompt_table[cmd]);
    } else {
        LOGMSG("%s: Non null current value is %e", __func__, *value_p);
        mvprintw(interactive_row - num_rows_adjust, 1, cmd_prompt_table[cmd], *value_p); 
    }
    clrtoeol();
    refresh();
}

/***************************************************************************** 
 *  io_get_doublevalue(double current_value)
 *
 *  Prompt the user for a double and return the value. On any user error 
 *  return current_value.
 *
 *  Warning: This call will block until the user enters a value. 
 * 
 *****************************************************************************/
double io_get_doublevalue(double current_value) 
{
    char * end_p;
    int curses_err; 
    bool input_err = false;
    double ret_value = 0;

    LOGFUNC();

    /* Get the double value */
    echo();
    curses_err = getnstr(input_buf, INPUT_BUF_SIZE);
    if (curses_err == ERR) {
       input_err = true; 
    }
    noecho();

    /* Check if it's safe to convert to a double */
    if (input_err == false) {
        errno = 0;
        ret_value = strtod(input_buf, &end_p);
    
        /* Check if strtod did not process all the input,
         * or the user entered nan or inf,
         * or the user entered a value that caused an underflow or overflow. 
         */
        if (('\0' !=  *end_p) || (isnan(ret_value)) || (isinf(ret_value)) || (errno == ERANGE)) {
            input_err = true;
        }
    }   

    /* If the user entered a negative number or there was an input error detected
     * process the invalid value.
     */
    if ((ret_value < 0) || (input_err)) {
        /* Notify user of invalid value */

        io_invalid();

        if (ret_value < 0) {
            LOGMSG("%s: User entered invalid negative value %e", __func__, ret_value);
        } else if (errno == ERANGE) {
            LOGMSG("%s: User entered overflow/underflow value", __func__);
        } else {
            LOGMSG("%s: User entered a value that cannot be converted: %s", __func__, input_buf);
        }

        ret_value = current_value;
    }

    io_restore_prompt();  

    LOGMSG("%s: Returning %f", __func__, ret_value);
    return ret_value;;

}

/***************************************************************************** 
 *  io_invalid()
 *
 *  Display the invalid message
 *
 *****************************************************************************/
void io_invalid()
{
    LOGFUNC();

    /* Notify user of invalid value */
    attron(A_REVERSE);
    mvprintw(interactive_row - num_rows_adjust, 1, "INVALID VALUE");
    attroff(A_REVERSE);
    clrtoeol();
    refresh();
    sleep(3);
      
}

/***************************************************************************** 
 *  io_restore_prompt()
 *
 *  Restore the prompt on the interactive command line
 *
 *****************************************************************************/
void io_restore_prompt()
{
    /* Restore the prompt */
    mvprintw(interactive_row - num_rows_adjust, 0, ">");
    clrtoeol();
    refresh();
}

