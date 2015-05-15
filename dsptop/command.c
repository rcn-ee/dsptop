/*
 * command.c
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
#include <getopt.h>
#include <ctype.h>
#include <limits.h>

#include "dsptop.h"
#include "command.h"
#include "io.h"
#include "error_handler.h"
#include "debug_log.h"


/***************************************************************************** 
 *  Internal Function Prototypes 
 *  - See Private Function section below for implementations
 *****************************************************************************/
static void string_to_argv(char * str, int * argc, char **argv_p);

/*****************************************************************************  
 *  Static Definitions and Initialization
 *****************************************************************************/
typedef enum {
    LONG_OPT_DELAY,
    LONG_OPT_FORMAT,
    LONG_OPT_HELP,
    LONG_OPT_LOGGING,
    LONG_OPT_MAXFILESIZE,
    LONG_OPT_NUM,
    LONG_OPT_OUT,
    LONG_OPT_PLOT_RESOLUTION,
    LONG_OPT_QUIET,
    LONG_OPT_SAMPLE,
    LONG_OPT_SYNC,
    LONG_OPT_TESTMODE,
    LONG_OPT_VERSION,
    LONG_OPT_PROCESS
} long_option_t;

static struct option long_options[] = {
    [LONG_OPT_DELAY] = {"delay", required_argument, 0,'d'},
    [LONG_OPT_FORMAT] = {"format", required_argument, 0, 'f'},
    [LONG_OPT_HELP] = {"help", no_argument, 0, 'h'},
    [LONG_OPT_LOGGING] = {"logging", required_argument, 0, 'l'},
    [LONG_OPT_MAXFILESIZE] = {"maxoutfile", required_argument, 0, 'm'},
    [LONG_OPT_NUM] = {"number", required_argument, 0, 'n'},
    [LONG_OPT_OUT] = {"output", required_argument, 0, 'o'},
    [LONG_OPT_PLOT_RESOLUTION] = {"resolution", required_argument, 0, 'r'},
    [LONG_OPT_QUIET] = {"quiet", no_argument, 0,'q'},
    [LONG_OPT_SAMPLE] = {"sample", required_argument, 0, 's'},
    [LONG_OPT_SYNC] = {"sync", no_argument, 0, 'y'},
    [LONG_OPT_TESTMODE] = {"testmode", required_argument, 0, '1'},
    [LONG_OPT_VERSION] = {"version", no_argument, 0, 'v'},
    [LONG_OPT_PROCESS] = {"process", no_argument, 0, 'p'}
};

static char * short_options = "d:f:hl:m:n:o:pqr:s:yv";

/* Max characters per line in the rc file */
#define MAX_LINE 256              

/* Format argument values */
static char * fmt_table[] = {
    [FMT_TEXT] = "text",
    [FMT_CSV] = "csv",
    [FMT_GNUPLOT] = "gnuplot",
    [FMT_GNUPLOT_WXT] = "gnuplot_wxt"
}; 

/* Logging argument values */
static char * log_table[] = {
    [LOG_FIRST] = "first",
    [LOG_LAST] = "last"
};

static char output_filename_path[PATH_MAX];

/***************************************************************************** 
 * Public Functions
 *
 *****************************************************************************/

/***************************************************************************** 
 * Parse argc/argv pair into options_t struct pointer
 * 
 * - Update option_t members that are updated by a command
 * - Execute simple commands immediately - like help/Version 
 *****************************************************************************/
void parse_args (int argc, char * const argv[], options_t * opt_p)
{

    while (1) {

        int option, option_index = 0;

        option = getopt_long(argc, argv, short_options, long_options, &option_index);
        
        /* No more options then all done */
        if (option == -1) break;

        switch (option) {
        case 'd':
            opt_p->delay_rate = atof(optarg);
            /* Adjust opt_p->delay_rate to >.5 && < 900 seconds. 
             * Note: adjust_delay() is a macro defined in command.h 
             */
            adjust_delay(opt_p->delay_rate); /* adjust_delay is a macro defined in command.h */

            LOGMSG("%s:Display update rate set to %f", __func__, opt_p->delay_rate);
            break;
        case 'f':
        {
            int i;
            for (i = 0; i < FMT_LAST; i++) {
                if (0 == strncmp(optarg, fmt_table[i], strlen(optarg))) {
                    break;
                }
            }

            if (i == FMT_LAST) {
                err_handler(ERR_TYPE_LOCAL, ERR_CMD_PARSER, "Invalid format argument");
                /* This return is to avoid Coverity issues, but in reality
                 * the err_handler will not return for fatal errors.
                 */
                return; 
            }

            opt_p->format_type = i;
            LOGMSG("%s:Format mode set to %s", __func__, optarg);
            break;
        }
        case '?': /* getopt did not understand the option */
        case 'h':
            fprintf(g_stdout, "\nUsage: %s [dhlmnpqrsSvy]\n", g_whoami);
            fprintf(g_stdout, "See \"man dsptop\" for more details.\n");
            fprintf(g_stdout, " --delay/-d <value>              Set the screen refresh rate in seconds (fractional seconds ok).\n");
            fprintf(g_stdout, " --format/-f <text|csv|gnuplot|gnuplot_wxt> \n");
            fprintf(g_stdout, "                                 Set the --output file format for DSP usage data, default is text.\n");
            fprintf(g_stdout, "                                 A file extension (.txt, .csv, .gp_cmd & .gp_dat) will be appended\n");
            fprintf(g_stdout, "                                 to the filename selected with --output. Selecting gnuplot_wxt will\n");
            fprintf(g_stdout, "                                 also launch gnuplot with a wxt terminal to plot DSP usage data.\n");
            fprintf(g_stdout, " --help/-h                       Display help.\n");
            fprintf(g_stdout, " --logging/-l <first|last>       Enable DSP message logging to stdout. DSP messages are captured in a\n");
            fprintf(g_stdout, "                                 circular buffer. Selecting \"first\" will write the DSP messages from\n");
            fprintf(g_stdout, "                                 the first full trace buffer to stdout and terminate dsptop. Selecting\n");
            fprintf(g_stdout, "                                 \"last\" forces the trace buffer to operate as a circular buffer in which\n");
            fprintf(g_stdout, "                                 case the logging data is written to stdout when dsptop is terminated.\n");
            fprintf(g_stdout, "                                 Use --output to select a file for logging rather than stdout.\n"); 
            fprintf(g_stdout, " --maxoutfile/-m <value>         Set the max size in bytes for --output files (default is 0 - infinite).\n");
            fprintf(g_stdout, " --number/-n <value>             Set the number of screen update iterations (default is 0 - infinite).\n");
            fprintf(g_stdout, " --output/-o <filename>          Select a file for logging (-l) or usage data (-f).\n");
            fprintf(g_stdout, " --resolution/-r <value>         Set plot resolution in seconds (fractional seconds ok).\n");
            fprintf(g_stdout, " --process/-p                    Switch from continuous DSP run/idle time accumulation to \n");
            fprintf(g_stdout, "                                 single DSP process run/idle accumulation. Run/idle time \n");
            fprintf(g_stdout, "                                 will stop advancing on exit of the current process and \n");
            fprintf(g_stdout, "                                 resume when the next process starts.\n");
            fprintf(g_stdout, " --quiet/-q                      Send all stdout to /dev/null except logging data.\n");
            fprintf(g_stdout, " --sample/-s <value>             Set the moving average sample window in seconds (fractional seconds ok).\n");
            fprintf(g_stdout, " --sync/-y                       Enable sync (used by dsptop_sync script).\n");
            fprintf(g_stdout, " --version/-v                    Display version.\n");
            fprintf(g_stdout, "\n                                 Ask a question or report a problem at e2e.ti.com.\n");
            fprintf(g_stdout, "\n");
            exit(0);
        case 'l':
        {

            int i;
            for (i = 0; i < LOG_END; i++) {
                if (0 == strncmp(optarg, log_table[i], strlen(optarg))) {
                    break;
                }
            }

            if (i == LOG_END) {
                err_handler(ERR_TYPE_LOCAL, ERR_CMD_PARSER, "Invalid Logging argument");
                /* This return is to avoid Coverity issues, but in reality
                 * the err_handler will not return for fatal errors.
                 */
                return; 
            }

            if (i== LOG_FIRST) {
                opt_p->stop_full = true;
                LOGMSG("%s:Stop logging when trace buffer is full", __func__);
            }

            opt_p->logging_enable = true;
            LOGMSG("%s:Logging enabled", __func__);
            break;
        }
        case 'm':
            opt_p->max_filesize = atoi(optarg);
            LOGMSG("%s:Maximum log file size set to %d", __func__, opt_p->max_filesize);
            break;
        case 'n':
            opt_p->num_iterations = atoi(optarg);
            LOGMSG("%s:Number of screen update iterations set to %d", __func__, opt_p->num_iterations);
            break;
        case 'r':
            opt_p->plot_resolution = atof(optarg);
            opt_p->plot_resolution = (opt_p->plot_resolution < .001) ? .001 : opt_p->plot_resolution;
            opt_p->plot_resolution = (opt_p->plot_resolution > 1) ? 1 : opt_p->plot_resolution;
            LOGMSG("%s:Plot resolution set to %f", __func__, opt_p->plot_resolution);
            break;
        case 'p':
            opt_p->process_until_exitmsg = true;
            LOGMSG("%s:Process mode enabled", __func__);
            break;
        case 'o':
        {
            if ((optarg != NULL) && (strlen(optarg) > 0)) {
                /* Most likely the user has not provided a filename and getopt_long has
                 * decided the next option is the filename.
                 */
                char invalid_set1[] = "-";
                if (strchr(invalid_set1, optarg[0])) {
                    char * msg = "Filename may not start with '-'";
                    LOGMSG("%s:%s", __func__, msg);
                    err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg); 
                }

                /* Trim leading and trailing whitespace.*/
                int i = strlen(optarg) - 1;
                while ((i >= 0) && isspace(optarg[i])) {
                    optarg[i--] = '\0';
                }
                i = 0;
                while (isspace(optarg[i])) {
                    optarg++;
                }
            }
            /* Error out if no filename left */
            if ((optarg == NULL) || (strlen(optarg) == 0)) {
                char * msg = "Filename cannot be \" \"";
                LOGMSG("%s:%s", __func__, msg);
                err_handler(ERR_TYPE_LOCAL, ERR_FATAL, msg);
            } 

            /* Save the filename since we can't be sure the source
             * will not be freed (as in the .dsptoprc case).
             */
            strncpy(output_filename_path, optarg, PATH_MAX);

            opt_p->out_filename_p = output_filename_path;
 
            LOGMSG("%s:File output to %s enabled", __func__,output_filename_path);

            break;
        }
        case 'q':
            opt_p->quiet_mode = true;
            g_stdout = fopen("/dev/null", "w");
            LOGMSG("%s:Quiet operation enabled", __func__);
            break;
        case 's':
            /* Adjust opt_p->mva_window if != 0 to >.5 && < 900 seconds. 
             * Note: adjust_mva_window() is a macro defined in command.h 
             */
            opt_p->mva_window = atof(optarg);
            adjust_mva_window(opt_p->mva_window);

            LOGMSG("%s:Sample window size set to %f", __func__, opt_p->mva_window);
            break;
        case 'y':
        {            
            opt_p->sync_enable = true; 
            LOGMSG("%s:sync enabled", __func__);
            break;
        }
        case 'v':
            fprintf(g_stdout, "%s: Version %d.%d.%d\n", g_whoami, g_major_version,
                    g_minor_version, g_patch_version);
            fprintf(g_stdout, "%s: Copyright (C) %d Texas Instruments, Inc.\n", 
                    g_whoami, g_copyright_year); 
            exit(0);
            break;
       case '1':
            /* Enable test mode */
            opt_p->test_mode = atoi(optarg);
            LOGMSG("%s:Test mode %d", __func__, opt_p->test_mode);
            break;
        } /* End of switch */
    } /* End of while */

    /* Must reset getopt_long so it restarts with element 1 of argv */
    optind = 1;

}

/***************************************************************************** 
 * Update option defaults from the resource file
 *
 * - Form the rc filename.
 * - Open the resource (rc) file. if it does not exist simply return.
 * - Process the rc file line by line.
 *   -  Convert the line string to a argv/argc list and parse it.
 *****************************************************************************/
void parse_rc(options_t * options_p)
{
    LOGFUNC();

    int argc = 0;
    char **argv = NULL;

    char * line;
    int offset;
    char * prefix = "dsptop ";

    LOGMSG("%s: Attempt to open %s to read options", __func__, options_p->rc_filename_p);

    /* Open the resource (rc) file. If it does not exist simply return. */
    FILE * fp = fopen(options_p->rc_filename_p,"r");
    if (fp == NULL) {
        LOGMSG("%s: RC file open for read failed", __func__);
        return;
    }

    /* Process the rc file line by line. */
    while (1) {
        /* Create a line buffer starting with "dsptop " since parse_args()  
         * assumes line came from the command line(skips argv[0]).
         */
        line = (char *)malloc(MAX_LINE);
        if (line == NULL) {
            fclose(fp);
            err_handler(ERR_TYPE_SYSTEM,ERR_MEM_ALLOC, NULL);
            /* This return is to avoid Coverity issues, but in reality
             * the err_handler will not return for fatal errors.
             */
            return;  
        }

        offset = strlen(prefix);
        memcpy(line, prefix, offset);

        if ( fgets(line + offset, MAX_LINE - offset, fp) == NULL) {
            free(line);
            break;
        }

        /* remove the newline */
        int len = strlen(line);
        if ((len > 0) && (line[len - 1] == '\n')) {
            line[len-1] = '\0';
        }

        LOGMSG("%s: Parsing RC file line:%s", __func__, line);

        string_to_argv(line, &argc, (char **)&argv);
        parse_args(argc, argv, options_p);

        /* All done with line and argv so need to free them.
         * Note: space for argv was malloced by string_to_argv.
         */
        free(line);
        /* Checking for NULL argv fools coverity into not
         * declaring a USE_AFTER_FREE defect.
         */
        if (argv != NULL) {
            free(argv);
        } 

    } /* End of while */

    fclose(fp);
    LOGMSG("%s: RC file for write closed", __func__);
}

/***************************************************************************** 
 * For each option update the resource file
 *
 * - Form the rc filename.
 * - Open the resource (rc) file. if it does not exist simply return.
 * - Process the rc file line by line.
 *   -  Convert the line string to a argv/argc list and parse it.
 *****************************************************************************/
void write_rc(options_t * options_p)
{
    LOGFUNC();

    LOGMSG("%s: Attempt to open %s to write options", __func__, options_p->rc_filename_p);

    /* Open the resource (rc) file. If it does not exist simply return. */
    FILE * fp = fopen(options_p->rc_filename_p,"w");
    if (fp == NULL) {
        LOGMSG("%s: RC file open for write failed", __func__);
        return;
    }

    if (!options_p->logging_enable) {
        fprintf(fp, "--%s=%.1f\n", long_options[LONG_OPT_DELAY].name,  options_p->delay_rate);
        fprintf(fp, "--%s=%.1f\n", long_options[LONG_OPT_SAMPLE].name,  options_p->mva_window);
    }

    if (options_p->num_iterations > 0) {
        fprintf(fp, "--%s=%d\n", long_options[LONG_OPT_NUM].name,  options_p->num_iterations);
    }
    
    if (options_p->process_until_exitmsg) {
        fprintf(fp, "--%s\n", long_options[LONG_OPT_PROCESS].name);
    }

    if (options_p->sync_enable) {
        fprintf(fp, "--%s\n", long_options[LONG_OPT_SYNC].name);
    }

    /* Logging */
    if (options_p->logging_enable) {
        fprintf(fp, "--%s=%s\n", long_options[LONG_OPT_LOGGING].name,  (options_p->stop_full) ? log_table[LOG_FIRST] : log_table[LOG_LAST] );
        fprintf(fp, "--%s=%s\n", long_options[LONG_OPT_OUT].name,  options_p->out_filename_p);
        fprintf(fp, "--%s=%d\n", long_options[LONG_OPT_MAXFILESIZE].name,  options_p->max_filesize);
    }

    /* Plottting */
    if ((!options_p->logging_enable) && (options_p->out_filename_p != NULL)) {
        fprintf(fp, "--%s=%s\n", long_options[LONG_OPT_FORMAT].name,  fmt_table[options_p->format_type] );
        fprintf(fp, "--%s=%s\n", long_options[LONG_OPT_OUT].name,  options_p->out_filename_p);
        fprintf(fp, "--%s=%.3f\n", long_options[LONG_OPT_PLOT_RESOLUTION].name,  options_p->plot_resolution);        
        fprintf(fp, "--%s=%d\n", long_options[LONG_OPT_MAXFILESIZE].name,  options_p->max_filesize);        
    }

    fclose(fp);
    LOGMSG("%s: RC file for write closed", __func__);

}

/***************************************************************************** 
 * get argument string
 *
 *****************************************************************************/
char * get_option_argument(char option, int arg_index)
{
    char * ret_p = NULL;

    switch (option) {
    case 'f':
        ret_p = fmt_table[arg_index];
        break;    
    default:
        err_handler(ERR_TYPE_LOCAL, ERR_CMD_PARSER, "Invalid option");
    }

    return ret_p; 
}
/***************************************************************************** 
 * Private Functions
 *
 *****************************************************************************/
/***************************************************************************** 
 * Convert string to argc/argv pair
 * 
 * - Make two copies of the string.
 * - Tokenize one to determine the number of elements
 * - Malloc an array of pointers.
 * - Tokenize the second copy into the pointer array.
 * Note: The client is responsible for freeing the space pointed to by argv.  
 *****************************************************************************/
static void string_to_argv(char * str, int * argc, char **argv_p)
{
    char *p;
    char **pp;
    int tokcnt = 0;

    if ( str == NULL ) {
        err_handler(ERR_TYPE_LOCAL, ERR_CMD_PARSER, "NULL string invalid");
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return; 
    }

    /* malloc space for a copy of the string */
    int char_cnt = strlen(str);

    /* increment to end of string */
    char_cnt++;

    char * copy1 = (char *)malloc(char_cnt);
    if (copy1 == NULL) {
        err_handler(ERR_TYPE_SYSTEM,ERR_MEM_ALLOC, NULL);
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return; 
    }

    memcpy(copy1, str, char_cnt);

    /* tokenize copy keeping count of pointers returned */
    p = strtok(copy1, " ");
    while(p) {
        tokcnt++;
        p= strtok(NULL, " ");

    }
    if (tokcnt == 0) {
        free(copy1);
        err_handler(ERR_TYPE_LOCAL, ERR_CMD_PARSER, NULL);
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return; 
    }    
    /* Add one more for the NULL pointer at the end or argv */
    *argc = tokcnt++;

    /* malloc space for argv array of pointers */
    *argv_p = malloc(sizeof(char *) * tokcnt);

    pp = (char **)*argv_p;
    if ( pp == NULL ) {
        free(copy1);
        err_handler(ERR_TYPE_SYSTEM,ERR_MEM_ALLOC, NULL);
        /* This return is to avoid Coverity issues, but in reality
         * the err_handler will not return for fatal errors.
         */
        return; 
    }
    
    /* tokenize str */
    {
        int i = 0;
        pp[i] = strtok(str, " ");
        while (pp[i] != NULL) {
            i++;
            pp[i] = strtok(NULL, " ");
        }
    }

    free(copy1); 

}


