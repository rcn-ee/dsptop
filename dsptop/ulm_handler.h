/*
 * stp_handler.h
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

#ifndef ULM_HANDLER_H
#define ULM_HANDLER_H

void ulm_init(options_t * options_p, dsp_type_t * dsp_p, dsp_usage_info_t * dsp_usage_info_p);
void ulm_close();
void ulm_config_etb_recording();
void ulm_start_etb_recording();
int ulm_stop_etb_recording();
void ulm_restart_etb_recording();
bool ulm_decode_to_logfile(char * log_filename_p, dsp_type_t * dsp_info_p);
void ulm_decode_usage();
void ulm_advance_usage(double newtime, double current, struct timeval * previous_tv, bool recording_stopped);
void ulm_generate_testdata();
void ulm_generate_testlarge();
void ulm_generate_testmsg();
void ulm_get_etb_state(int * etb_percent_full, bool * is_etb_enabled, bool * is_etb_wrapped, bool * is_logging, bool * stop_trigger);
void ulm_get_gaptime(struct timeval * gap_tv_p);
void ulm_clr_gaptime();
void ulm_log_lastusage();

#endif/* ULM_HANDLER_H */


