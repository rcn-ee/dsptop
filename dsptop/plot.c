/*
 * plot.c
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>

#include "command.h"
#include "error_handler.h"
#include "error.h"
#include "debug_log.h"
#include "io.h"
#include "utility.h"
#include "temp.h"
#include "plot.h"

/***************************************************************************** 
 *  Internal Function Prototypes 
 *  - See Private Function section below for implementations )
 *****************************************************************************/
static void plot_start_gnuplot();
static void plot_postprocess_gnuplot();
static void plot_launch_gnuplot();

/*****************************************************************************  
 *  Static Definitions and Initialization
 *****************************************************************************/
static bool plot_enable = false;
static bool plot_process_mode = false;
static bool plot_no_data = false;
static format_type_t plot_format = FMT_TEXT;
static char * plot_filename_p = NULL;
static FILE * plot_fp = NULL;
static FILE * cmd_fp = NULL;
static FILE * tmp_plot_fp = NULL;
static uint32_t plot_maxfilesize = 0;
static int plot_num_dsps = 0;
static int plot_fifo_fd = -1;

static const char * tmp_filename_p = "/tmp/dsptop_plot.gnuplot";

static char * gnuplot_color_table[] = {
    "red",
    "blue",
    "violet",
    "coral",
    "navy",
    "turquoise",
    "green",
    "magenta",
    "orange",
    "gray",
    "black",
    "royalblue",
    "cyan",
    "skyblue",
    "gray50",
    "purple"
};

static FILE * gnuplot_fp = NULL;

const int maxfile_record_size = 1024;

static char datafile_name_p[PATH_MAX];
static char cmdfile_name_p[PATH_MAX];

/***************************************************************************** 
 * Public Functions
 *
 *****************************************************************************/

/***************************************************************************** 
 *  plot_init()
 *
 *  Initialize plot file generation with usage data
 *
 *****************************************************************************/ 
void plot_init(options_t * options_p, dsp_type_t * dsp_info_p)
{
    LOGFUNC();

    /* If logging enabled or the out file is set, then plotting enabled */
    if ((options_p->logging_enable) || (options_p->out_filename_p == NULL)) {
        return;
    }

    /* Initialize plot parameters */
    plot_filename_p = options_p->out_filename_p;
    plot_format = options_p->format_type;
    plot_maxfilesize = options_p->max_filesize;
    plot_num_dsps = dsp_info_p->num_dsps;
    plot_process_mode = options_p->process_until_exitmsg;

    /* Open files required to plot */
    plot_enable = true;
    plot_start();

    /* If supported write plot commands to file*/
    switch (plot_format) {
        case FMT_GNUPLOT_WXT:
        case FMT_GNUPLOT:
        {
            /* Open and generate the gnuplot command file */
            util_gen_filename(cmdfile_name_p, PATH_MAX, options_p->out_filename_p, ".gp_cmd");
  
            cmd_fp = fopen(cmdfile_name_p, "w");
            if (cmd_fp == NULL) {
                char * msg = "Can not open gnuplot command file for writing";
                LOGMSG("%s:%s: %s", __func__, msg, options_p->out_filename_p);
                err_handler(ERR_TYPE_SYSTEM, ERR_FATAL, msg);
                return;   /* Dummy return to keep code checker from declaring issue */
            }

            int graph_num = (temp_is_enabled()) ? plot_num_dsps + 1 : plot_num_dsps;

            fprintf(cmd_fp, "set terminal wxt size 1125,786 font 'arial,7' persist\n");
            fprintf(cmd_fp, "set multiplot layout %d, 1 title \"DSP Usage %% Plot\"\n", graph_num);
            fprintf(cmd_fp, "set datafile separator \",\"\n");
            fprintf(cmd_fp, "set autoscale\n");
            fprintf(cmd_fp, "set border lw .5\n");

            for ( int i = 0; i < plot_num_dsps; i++) {

                fprintf(cmd_fp,"plot \"%s\" index %d using 1:2 with linespoints lt rgb \"%s\" lw .8 pt 1 title \"DSP_%d Usage %%\"\n", 
                        datafile_name_p, i, gnuplot_color_table[i], i);

            }

            if (temp_is_enabled()) {
                fprintf(cmd_fp,"plot \"%s\" index %d using 1:2 with linespoints lt rgb \"%s\" lw .8 pt 1 title \"Temperature\"\n",
                        datafile_name_p, plot_num_dsps, gnuplot_color_table[plot_num_dsps]);    
            }

            fprintf(cmd_fp, "unset multiplot\n");
            fprintf(cmd_fp, "pause -1\n");

            if (plot_process_mode) {
                fprintf(cmd_fp, "exit\n");
            }

            fclose(cmd_fp); 
            break;
        }
        case FMT_TEXT:
        case FMT_CSV:
        case FMT_FIFO:
        default:
            break; /* Nothing to initialize */
    }
}

/***************************************************************************** 
 *  plot_close()
 *
 *****************************************************************************/ 
void plot_close()
{
    LOGFUNC();

    if (plot_enable == false) {
        return;
    }

    if (plot_fp != NULL) {
        fclose(plot_fp);
    }

    if (gnuplot_fp != NULL) {
        LOGMSG("%s: Closing gnuplot_fp", __func__);
        fprintf(gnuplot_fp, "e\nexit gnuplot\n");
        pclose(gnuplot_fp);
    }
}

/***************************************************************************** 
 *  plot_write_current()
 *
 *****************************************************************************/
void plot_write_current(int dsp_index, double timestamp, double runtime, double idletime, int8_t temp)
{

    char msg_buf[maxfile_record_size];

    if (plot_enable == false) {
        return;
    }

    if (plot_fp == NULL && plot_fifo_fd == -1) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    /* Select the format of the data (note: will not display temp if it's not available)
     * Plot: timestamp, DSP_Index, usage% temperature
     */
    char * format_p = NULL;
    switch (plot_format) {
        case FMT_TEXT:
            format_p = (temp == -128) ? "%-9.3f %-3d %-7.2f   %-9.6f   %-9.6f\n" : "%-9.3f %-3d %-7.2f   %-9.6f   %-9.6f   %-d\n";
            break;
        case FMT_GNUPLOT_WXT:
        case FMT_GNUPLOT: 
        case FMT_CSV:
            format_p = (temp == -128) ? "%4.3f,%d,%3.2f,%6.6f,%6.6f\n" : "%4.3f,%d,%3.2f,%6.6f,%6.6f,%d\n";
            break;
        case FMT_FIFO:
            format_p = "CPULOAD: DSP%d %d\n";
            break;
        default:
            err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    /* Form the message in msg_buf */
    size_t chr_cnt = 0;
    double usage = (runtime == 0) ? 0 : (runtime/(runtime + idletime)) * 100.0;

    if (plot_format == FMT_FIFO) {
        chr_cnt = snprintf(msg_buf, maxfile_record_size, format_p, dsp_index+1, (int) usage);
        write(plot_fifo_fd, msg_buf, chr_cnt);
        return;
    }

    chr_cnt += snprintf(msg_buf + chr_cnt, maxfile_record_size - chr_cnt, format_p, timestamp, dsp_index, usage, runtime, idletime, temp);
    if (maxfile_record_size - chr_cnt < 1) {
        err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    } 

    /* If write will cause file size to be exceeded, then return */
    if (plot_maxfilesize > 0) {
        size_t file_size = util_get_filesize(plot_filename_p);
        if ((chr_cnt + file_size) > plot_maxfilesize) {
            LOGMSG("%s: Max file size for plot file exceeded", __func__);
            return;
        }
    }

    /* Write msg_buf to the file */
    size_t fwrite_cnt = fwrite(msg_buf, 1, chr_cnt, plot_fp);
    if (fwrite_cnt != chr_cnt) {
        char * msg = "Error while writing plot file";
        LOGMSG("%s:%s: %d", __func__, msg, fwrite_cnt);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
    }

}
/***************************************************************************** 
 *  plot_start()
 *
 *  Open the files required for plotting. Plot_start can be called 
 *  after post processing to reopen files.
 *
 * Note: postprocess closes all files after putting data in final format.
 *****************************************************************************/ 
void plot_start()
{
    LOGFUNC();

    if (plot_enable == false) {
        return;
    }

    if (plot_fp != NULL) {
        fclose(plot_fp);
    }

    /* Select the file type */
    char * file_ext = ".txt";
    switch (plot_format) {
        case FMT_GNUPLOT_WXT:
        case FMT_GNUPLOT:
            file_ext = ".gp_dat";
            if (tmp_plot_fp != NULL) {
                fclose(tmp_plot_fp);
            }
            break; 
        case FMT_CSV:
            file_ext = ".csv";
            break;
        case FMT_TEXT:
            break;
        case FMT_FIFO:
            file_ext = "";
            break;
        default:
            err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

    /* Open the data file */
    /* Note: the FIFO format writes to a FIFO instead of a regular file, the */
    /*       FIFO output format is: "CPULOAD DSP# usage_percentage", the     */
    /*       expected consumer of FIFO is soc-performance-monitor tool.      */
    util_gen_filename(datafile_name_p, PATH_MAX, plot_filename_p, file_ext);

    if (plot_format != FMT_FIFO)
        plot_fp = fopen(datafile_name_p, "w");
    else
        plot_fifo_fd = open(datafile_name_p, O_WRONLY | O_NONBLOCK);

    if (plot_fp == NULL && plot_fifo_fd == -1) {
        char * msg = "Can not open out file for writing";
        LOGMSG("%s:%s: %s", __func__, msg, datafile_name_p);
        err_handler(ERR_TYPE_SYSTEM, ERR_FATAL, msg);
        return;   /* Dummy return to keep code checker from declaring issue */
    } else {
        LOGMSG("%s: Plotting enabled to: %s", __func__, datafile_name_p);
    }

    switch (plot_format) {
        case FMT_GNUPLOT_WXT:
        case FMT_GNUPLOT:
        /* In the gnuplot case create a temporary file for data that 
           is post processed to format properly for gnuplot.
         */
            plot_start_gnuplot();
            break;
        /* In the text and csv cases put a header in the data file */
        case FMT_TEXT:
        {
            char * format_p = (temp_is_enabled()) ? "Timestamp DSP Usage %%   Run Time    Idle Time   Temperature\n"
                                                  : "Timestamp DSP Usage %%   Run Time    Idle Time\n";
            fprintf(plot_fp, format_p);
            break;
        }
        case FMT_CSV:
        {
            char * format_p = (temp_is_enabled()) ? "Timestamp,DSP Usage %%,Run Time,Idle Time,Temperature\n"
                                                  : "Timestamp,DSP Usage %%,Run Time,Idle Time\n";
            fprintf(plot_fp, format_p);
            break;
        }
        case FMT_FIFO:
        {
            char msg_buf[32];
            for (int i = 0; i < plot_num_dsps; i++)
            {
                int chr_cnt = snprintf(msg_buf, 32, "CPULOAD: DSP%d 0\n", i+1);
                write(plot_fifo_fd, msg_buf, chr_cnt);
            }
            break;
        }
        default:
            err_handler(ERR_TYPE_LOCAL, ERR_DEBUG, NULL);
    }

}


/***************************************************************************** 
 *  plot_launch()
 *
 *  Launch the plot
 *****************************************************************************/ 
void plot_launch()
{
    LOGFUNC();

    if (plot_enable == false) {
        return;
    }

    switch (plot_format) {
        case FMT_GNUPLOT_WXT:
            plot_launch_gnuplot();
            break;
        case FMT_TEXT: 
        case FMT_CSV:
        case FMT_GNUPLOT:
        case FMT_FIFO:
        default:
            break; /* Nothing to launch */
    }

}

/***************************************************************************** 
 *  plot_postprocess()
 *
 *  If post processing required
 *****************************************************************************/ 
void plot_postprocess()
{
    LOGFUNC();

    if (plot_enable == false) {
        return;
    }

    switch (plot_format) {
        case FMT_GNUPLOT_WXT:
        case FMT_GNUPLOT:
            plot_postprocess_gnuplot();
            break;
        case FMT_FIFO:
            if (plot_fifo_fd != -1)  close(plot_fifo_fd);
            break;
        case FMT_TEXT: 
        case FMT_CSV:
        default:
            /* Nothing to post process in text and csv case so simply close the out file */
            if (plot_fp != NULL) {
                fclose(plot_fp);
                plot_fp = NULL;
            }
            break; 
    }

}


/***************************************************************************** 
 * Private Functions
 *
 * Each plot type will have a set of static plot support functions:
 *   plot_start_n()
 *   plot_postprocess_n() - Post process the data (if required)
 *   plot_launch_n() - Launch the plotting sw (if supported)
 *
 *****************************************************************************/

/***************************************************************************** 
 *  plot_start_gnuplot()
 *
 *****************************************************************************/ 
static void plot_start_gnuplot()
{
    LOGFUNC();

    /* Open tmp file */           
    tmp_plot_fp = fopen(tmp_filename_p, "w");

    if (tmp_plot_fp == NULL) {
        char * msg = "Could not open /tmp/dsptop_plot.gnuplot for writing";
        LOGMSG("%s:%s", __func__, msg);
        err_handler(ERR_TYPE_SYSTEM, ERR_FATAL, msg);
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return;
    }

    /* chmod so the next call to fopen(tmp_filename_p) will not fail if the tmp file still exists. */ 
    if (chmod(tmp_filename_p, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH ) < 0) {
        char * msg = "Could not change /tmp/dsptop_plot.gnuplot file permissions";
        LOGMSG("%s:%s", __func__, msg);
        err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return;
    }

    /* Exchange file pointer with tmp */
    FILE * plot_fp_sav = plot_fp;                        
    plot_fp = tmp_plot_fp;            
    tmp_plot_fp = plot_fp_sav;

    LOGMSG("%s:Opened file %s for gnuplot data", __func__, tmp_filename_p);

}

/***************************************************************************** 
 *  plot_postprocess_gnuplot()
 *
 *****************************************************************************/ 
static void plot_postprocess_gnuplot()
{
    LOGFUNC();

    /* Exchange file pointers */
    FILE * plot_fp_sav = plot_fp;                        
    plot_fp = tmp_plot_fp;            
    tmp_plot_fp = plot_fp_sav;

    if ((tmp_plot_fp == NULL) || (plot_fp == NULL)) {
        plot_no_data = true;
        if (tmp_plot_fp == NULL) {
            LOGMSG("%s: tmp_plot_fp is NULL", __func__);
        }
        if (plot_fp == NULL) {
            LOGMSG("%s: plot_fp is NULL", __func__);
        }
        return;
    }

    /* Reopen the temp file for reading */
    fclose(tmp_plot_fp);
    tmp_plot_fp = fopen(tmp_filename_p, "r");  
    if (tmp_plot_fp == NULL) {
        char * msg = "Could not open /tmp/dsptop_plot.gnuplot for reading";
        LOGMSG("%s:%s", __func__, msg);
        err_handler(ERR_TYPE_SYSTEM, ERR_FATAL, msg);
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return;
    }    

    LOGMSG("%s: tmp file opened for reading", __func__);

    /* Flag to indicate dsp index comment has been written */
    bool dsp_index_set[plot_num_dsps];
    memset(dsp_index_set, 0, sizeof(dsp_index_set));

    /* Track number of samples written to out (.gp_dat) file */
    int sample_cnt = 0;
    bool temp_enabled = temp_is_enabled();
    char * rdformat_p = (temp_enabled) ? "%f,%d,%f,%f,%f,%d" : "%f,%d,%f";
    bool temp_index_set = false;
    double previous_timestamp = -1.0;    

    int graph_num = (temp_enabled) ? plot_num_dsps + 1 : plot_num_dsps;

    for ( int i = 0; i < graph_num; i++) {

        do {

            float timestamp = 0;
            int dsp_index= 0;
            float usage= 0;
            double runtime = 0;
            double idletime = 0;
            int temp = 0;

            char line[maxfile_record_size];

            char * ret = fgets(line, sizeof line, tmp_plot_fp);

            /* Check for EOF */
            if (ret == NULL) {
                /* Gnuplot requires two line breaks between data sets */
                fprintf(plot_fp,"\n\n");
                /* Rewind the file */
                rewind(tmp_plot_fp);
                break;
            }

            int item_cnt = sscanf(line,rdformat_p, &timestamp, &dsp_index, &usage, &runtime, &idletime, &temp);

            if ((!temp_enabled && (item_cnt != 3)) || (temp_enabled && (item_cnt != 6))) { 
                char * msg = "Error reading from /tmp/dsptop_plot.gnuplot";
                LOGMSG("%s:%s", __func__, msg);
                err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            }

            /* Scan for dsp usage data */
            if (dsp_index == i) {

                if (dsp_index_set[i] == false) {

                    /* Add a comment so we can identify the data in the file */
                    fprintf(plot_fp, "# DSP_%d\n", i);
                    dsp_index_set[i] = true;
                }

                fprintf(plot_fp, "%.3f, %.2f\n", timestamp, usage);
                sample_cnt++;
            }

            /* Scan for temperature data */
            if ((temp_enabled) && (i == plot_num_dsps)) {

                if (temp_index_set == false) {
                    fprintf(plot_fp, "# Temperature in C\n");
                    temp_index_set = true;
                }
                
                if (timestamp > previous_timestamp) {
                    fprintf(plot_fp, "%.3f, %d\n", timestamp, temp);
                    sample_cnt++;
                    previous_timestamp = timestamp;
                }
            }

        } while (1);

    }

    if (sample_cnt == 0) {
        LOGMSG("%s: No data samples found in tmp file", __func__);
        plot_no_data = true;
    } else {
        LOGMSG("%s: Found %d samples in tmp file", __func__, sample_cnt);
        plot_no_data = false;
    }
        
    /* Remove tmp file and close the fp */
    fclose(tmp_plot_fp);
    tmp_plot_fp = NULL;
    if (remove(tmp_filename_p) < 0) {
        LOGMSG("%s: Could not remove %s file, errno is %d", __func__, tmp_filename_p, errno);  
    }

    /* Close the data file */
    fclose(plot_fp);
    plot_fp = NULL;


}

/***************************************************************************** 
 *  plot_launch_gnuplot()
 *
 *****************************************************************************/ 
static void plot_launch_gnuplot()
{
    LOGFUNC();

    if (plot_no_data == true) {
        LOGMSG("%s: no data to plot", __func__);
        return;
    }
        
    if (gnuplot_fp == NULL) {
        /* Start gnuplot with both stdout and stderr redirected to a file */
        gnuplot_fp = popen("gnuplot 1>gnuplot_out.txt 2>&1", "w");
        if (gnuplot_fp == NULL) {
            LOGMSG("%s: Can not popen gnuplot", __func__);
            return;
        }
        LOGMSG("%s: Opened gnuplot for stdin input", __func__);
    } else {

        /* If file has gone bad return - like user terminated gnuplot */
        int gnuplot_fd = fileno(gnuplot_fp);
        if ((gnuplot_fd == -1) && (errno == EBADF)) {
            LOGMSG("%s: gnuplot pipe has gone bad", __func__);
            return;
        }            

        int gnuplot_fd_flags = fcntl(gnuplot_fd, F_GETFL);
        LOGMSG("%s: gnuplot_fd_flags is %d", __func__, gnuplot_fd_flags);
        if ((gnuplot_fd_flags == -1) && (errno == EBADF)) {
            return;
        }

        LOGMSG("%s: Send gnuplot clear cmd", __func__);

        fprintf(gnuplot_fp, "\nclear\n");
        fflush(gnuplot_fp);          
    }

    /* Send the commands */
    LOGMSG("%s: Sending commands to gnuplot", __func__);        

    int graph_num = (temp_is_enabled()) ? plot_num_dsps + 1 : plot_num_dsps;

    fprintf(gnuplot_fp, "set terminal wxt size 1125,786 font 'arial,7' persist\n");
    fprintf(gnuplot_fp, "set multiplot layout %d, 1 title \"DSP Usage %% Plot\"\n", graph_num);
    fprintf(gnuplot_fp, "set datafile separator \",\"\n");
    fprintf(gnuplot_fp, "set autoscale\n");
    fprintf(gnuplot_fp, "set border lw .5\n");

    plot_fp = fopen(datafile_name_p, "r");
    if (plot_fp == NULL) {
        char * msg = "Can not open gnuplot data file for reading";
        LOGMSG("%s:%s: %s", __func__, msg, datafile_name_p);
        err_handler(ERR_TYPE_SYSTEM, ERR_FATAL, msg);
        return;   /* Dummy return to keep code checker from declaring issue */
    }

    for (int i = 0; i < graph_num; i++) {  

        if (i < plot_num_dsps) {
            fprintf(gnuplot_fp,"plot '-' using 1:2 with linespoints lt rgb \"%s\" lw .8 pt 1 title \"DSP_%d Usage %%\"\n", gnuplot_color_table[i], i);
        }

        if ((temp_is_enabled()) && (i == plot_num_dsps)) { 
            fprintf(gnuplot_fp,"plot '-' using 1:2 with linespoints lt rgb \"%s\" lw .8 pt 1 title \"Temperature\"\n", gnuplot_color_table[i]);
        }

        /* Send the data in the current datafile_name_p*/
        LOGMSG("%s: Sending data to gnuplot", __func__);

        char line [maxfile_record_size];
        bool skip = false;
        while (fgets(line, sizeof line, plot_fp) != NULL) {

            /* If record starts with '/n' then send "e" to mark end of the data for this DSP */
            if ((line[0] == '\n') && (skip == false)) {
                fprintf(gnuplot_fp, "e\n");
                skip = true;
                LOGMSG("%s: Skip first lf", __func__);
                continue;
            }

            /* Break after second line feed found */
            if ((line[0] == '\n') && (skip == true)) {
                LOGMSG("%s: Skip 2nd lf", __func__);
                break;
            }

            /* Skip comments - send data to gnuplot */
            if (line[0] != '#') {
                char * lp = line;
                fprintf(gnuplot_fp, "%s", lp);
            } 

        } /* End of while loop */

    } /* End of for loop */

    fprintf(gnuplot_fp, "unset multiplot\n");
    fprintf(gnuplot_fp, "pause -1\n");

    fflush(gnuplot_fp); 

    fclose(plot_fp);
    plot_fp = NULL;

}
