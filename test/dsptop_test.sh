#!/bin/bash
#----------------------------------------------------------------------
# Automated test script to test dsptop.
#----------------------------------------------------------------------

MATCH=
VERIFY=
DEVICE=C66AK2Hxx
TEST_FAILURES=()
TEST_SHORT_SUMMARY=()
TEST_SUMMARY=()
CURRENT_ITERATION=0

while [[ $# > 1 ]]
do
key="$1"
shift

case $key in
    -d|--device)
    DEVICE="$1"
    shift
    ;;
    -i|--iterations)
    ITERATIONS="$1"
    shift
    ;;
    -m|--match)
    MATCH="$1"
    shift
    ;;
    -v|--verify)
    VERIFY="$1"
    shift
    ;;
    *)
            # unknown option
    ;;
esac
done

if [ "$MATCH" != "" ] && [ "$VERIFY" != "" ]; then
    echo Invalid arugments. Do not use --match and --verify concurrently.
fi

# remove any existing log files
failure_file="errors.log"
summary_file="summary.log"
rm -f $failure_file
rm -f $summary_file


write_rc_file()
{
    declare -a rc=("${!1}") 
    rc_file=$2
    
    rm -f $rc_file   
 
    #echo "Writing rc_file $rc_file."
    for j in "${rc[@]}"
    do
        printf "%s\n" "$j" >> $rc_file
	#echo "$j"
    done
}

#----------------------------------------------------------------------
# verify_single_log will compare idle and run times for each 
# DSP to an expected value for the given example. If the difference
# between the idle and run time is below a given threshold, the 
# test will pass.
#----------------------------------------------------------------------
verify_ret_value="null error"
verify_single_log()
{
    declare -a expected_values=("${!1}")
    local inputdata=$2
    local run_thresh=$3
    local idle_thresh=$4
    local ext_mem_code_total_exp=`echo $5 | cut -d "," -f1`
    local ext_mem_code_used_exp=`echo $5 | cut -d "," -f2`
    local ext_mem_data_total_exp=`echo $5 | cut -d "," -f3`
    local ext_mem_data_used_exp=`echo $5 | cut -d "," -f4`
    local int_mem_data_total_exp=`echo $5 | cut -d "," -f5`
    local int_mem_data_used_exp=`echo $5 | cut -d "," -f6`
    local ext_mem_code_total_thresh=`echo $6 | cut -d "," -f1`
    local ext_mem_code_used_thresh=`echo $6 | cut -d "," -f2`
    local ext_mem_data_total_thresh=`echo $6 | cut -d "," -f3`
    local ext_mem_data_used_thresh=`echo $6 | cut -d "," -f4`
    local int_mem_data_total_thresh=`echo $6 | cut -d "," -f5`
    local int_mem_data_used_thresh=`echo $6 | cut -d "," -f6`
    local sample_window=$7
    local elapsed_thresh=$8
    local run_fail=0
    local idle_fail=0
    local zeros_fail=0
    local elapsed_fail=0
    local run_percent=0
    local idle_percent=0
    local procnum_exp=0
    local run_time_exp=0
    local idle_time_exp=0	
    local procnum_inp=0
    local run_time_inp=0
    local idle_time_inp=0
    local run_diff=0
    local idle_diff=0
    local run_diff_run1=0
    local idle_diff_run1=0
    local run_times_run1=()
    local idle_times_run1=()
    local result=0
    local first_failure=0
    local mem_diff=0
    local elapsed_diff=0

    declare -a input_data_array
    # generate array of input values from log file

    verify_ret_value="null error"

    while read p; do
        # remove everything before |
        inputline=`echo $p | sed 's/^.*|/|/'`
        if [[ $inputline == *"Num dsps is"* ]]; then
            numdsps=`echo $inputline | sed 's/.*Num dsps is \([0-9]*\).*/\1/'`
            if [[ $numdsps != ${#expected_values[@]} ]]; then
		verify_ret_value=$verify_ret_value:"Validation expecting ${#expected_values[@]} DSPs."
            fi
        elif [[ $inputline == *"External Mem Code"* ]]; then
            ext_mem_code_total=`echo "$inputline" | cut -d "," -f1`
	    ext_mem_code_total=`echo $ext_mem_code_total | sed 's/.*Data \([0-9]*\).*/\1/'`
            ext_mem_code_used=`echo "$inputline" | cut -d "," -f2`
            ext_mem_code_used=`echo "$ext_mem_code_used" | cut -d "%" -f1`
	    
            mem_diff=$(awk "BEGIN{x = ($ext_mem_code_total_exp - $ext_mem_code_total); print x < 0 ? -x : x}")
	    result=$(awk "BEGIN{$mem_diff > $ext_mem_code_total_thresh ? x=1 : x=0; print x}")
	    if [[ $result == 1 ]]; then
	    	verify_ret_value=$verify_ret_value:"External Mem Code/Data Total exceeds threshold for expected value"
	    fi
            mem_diff=$(awk "BEGIN{x = ($ext_mem_code_used_exp - $ext_mem_code_used); print ((x < 0) ? -x : x)}")
	    result=$(awk "BEGIN{$mem_diff > $ext_mem_code_used_thresh ? x=1 : x=0; print x}")
	    if [[ $result == 1 ]]; then
	    	verify_ret_value=$verify_ret_value:"External Mem Code/Data % Used exceeds threshold for expected value"
	    fi
        elif [[ $inputline == *"External Mem Data"* ]]; then
            ext_mem_data_total=`echo "$inputline" | cut -d "," -f1`
	    ext_mem_data_total=`echo $ext_mem_data_total | sed 's/.*Data \([0-9]*\).*/\1/'`
            ext_mem_data_used=`echo "$inputline" | cut -d "," -f2`
            ext_mem_data_used=`echo "$ext_mem_data_used" | cut -d "%" -f1`
	    
            mem_diff=$(awk "BEGIN{x = ($ext_mem_data_total_exp - $ext_mem_data_total); print x < 0 ? -x : x}")
	    result=$(awk "BEGIN{$mem_diff > $ext_mem_data_total_thresh ? x=1 : x=0; print x}")
	    if [[ $result == 1 ]]; then
	    	verify_ret_value=$verify_ret_value:"External Mem Data Total exceeds threshold for expected value"
	    fi
            mem_diff=$(awk "BEGIN{x = ($ext_mem_data_used_exp - $ext_mem_data_used); print ((x < 0) ? -x : x)}")
	    result=$(awk "BEGIN{$mem_diff > $ext_mem_data_used_thresh ? x=1 : x=0; print x}")
	    if [[ $result == 1 ]]; then
	    	verify_ret_value=$verify_ret_value:"External Mem Data % Used exceeds threshold for expected value"
	    fi
        elif [[ $inputline == *"Internal Mem Data"* ]]; then
            int_mem_data_total=`echo "$inputline" | cut -d "," -f1`
	    int_mem_data_total=`echo $int_mem_data_total | sed 's/.*Data \([0-9]*\).*/\1/'`
            int_mem_data_used=`echo "$inputline" | cut -d "," -f2`
            int_mem_data_used=`echo "$int_mem_data_used" | cut -d "%" -f1`
	    
            mem_diff=$(awk "BEGIN{x = ($int_mem_data_total_exp - $int_mem_data_total); print x < 0 ? -x : x}")
	    result=$(awk "BEGIN{$mem_diff > $int_mem_data_total_thresh ? x=1 : x=0; print x}")
	    if [[ $result == 1 ]]; then
	    	verify_ret_value=$verify_ret_value:"Internal Mem Data Total exceeds threshold for expected value"
	    fi
            mem_diff=$(awk "BEGIN{x = ($int_mem_data_used_exp - $int_mem_data_used); print ((x < 0) ? -x : x)}")
	    result=$(awk "BEGIN{$mem_diff > $int_mem_data_used_thresh ? x=1 : x=0; print x}")
	    if [[ $result == 1 ]]; then
	    	verify_ret_value=$verify_ret_value:"Internal Mem Data % Used exceeds threshold for expected value"
	    fi
        elif [[ $inputline == *"ARM Freq"* ]]; then
            armfreq=`echo "$inputline" | cut -d "," -f1`
	    armfreq=`echo $armfreq | sed 's/.*Freq \([0-9]*\).*/\1/'`
            dspfreq=`echo "$inputline" | cut -d "," -f2`
	    dspfreq=`echo $dspfreq | sed 's/.*Freq \([0-9]*\).*/\1/'`
	    if [[ $armfreq == 0 ]]; then
		verify_ret_value=$verify_ret_value:"ARM frequency incorrectly reported as 0 or null."
	    fi
	    if [[ $dspfreq == 0 ]]; then
		verify_ret_value=$verify_ret_value:"DSP frequency incorrectly reported as 0 or null."
	    fi
        elif [[ $inputline == *"emperature"* ]]; then
	    # Do not check temperatures if sensor isn't calibrated or using sudo build.
	    if [[ $inputline != *"sensor not calibrated"* ]]; then
		current_temp=`echo $inputline | cut -d "," -f1`
		current_temp=`echo $current_temp | sed 's/.*Temperature \([0-9]*\).*/\1/'`
		max_temp=`echo $inputline | cut -d "," -f2`
		max_temp=`echo $max_temp | sed 's/.*Temperature \([0-9]*\).*/\1/'`
		# assume temperatures are at least room temeprature (20C) and below max limit (100C)
		if [ $current_temp -lt 20 ] || [ $current_temp -gt 100 ]; then
		    verify_ret_value=$verify_ret_value:"Current temperature (${current_temp}C) below or above testing threshold (20C - 100C)."
		fi
		if [ $max_temp -lt 20 ] || [ $max_temp -gt 100 ]; then
		    verify_ret_value=$verify_ret_value:"Max temperature (${max_temp}C) below or above testing threshold (20C - 100C)."
		fi
	    fi
        elif [[ $inputline == *"DSP_"* ]]; then
	    # fill array with data points <procnum> <run_time> <idle_time>
	    pnum=`echo $inputline | sed 's/.*DSP_\([0-9]*\).*/\1/'`
	    runt=`echo "$inputline" | cut -d " " -f5`
	    idlet=`echo "$inputline" | cut -d " " -f8`
	    
	    input_data_array[$pnum]="$pnum $runt $idlet" 
        fi    
    done <$inputdata


    # First check if all input and expected values are zero.
    for ((n=0;n<${#expected_values[@]};++n)); do
        procnum_exp=`echo "${expected_values[n]}" | cut -d " " -f1`

        run_time_exp=`echo "${expected_values[n]}" | cut -d " " -f2`
        result=$(awk "BEGIN{$run_time_exp == 0 ? x=0 : x=1; print x}")
        nonzero_count_expected=$(($nonzero_count_expected + $result))

        idle_time_exp=`echo "${expected_values[n]}" | cut -d " " -f3`    
        result=$(awk "BEGIN{$idle_time_exp == 0 ? x=0 : x=1; print x}")
        nonzero_count_expected=$(($nonzero_count_expected + $result))

        procnum_inp=`echo "${input_data_array[n]}" | cut -d " " -f1`

        run_time_inp=`echo "${input_data_array[n]}" | cut -d " " -f2`
        result=$(awk "BEGIN{$run_time_inp == 0 ? x=0 : x=1; print x}")
        nonzero_count_input=$(($nonzero_count_input + $result))

        idle_time_inp=`echo "${input_data_array[n]}" | cut -d " " -f3`
        result=$(awk "BEGIN{$idle_time_inp == 0 ? x=0 : x=1; print x}")
        nonzero_count_input=$(($nonzero_count_input + $result))
    done

    zero_fail=0
    if [ $nonzero_count_expected == 0 ] && [ $nonzero_count_input != 0 ]; then
        verify_ret_value=$verify_ret_value:"All expected values are zero.  Test output contains non-zero values."
	zero_fail=1
    elif [ $nonzero_count_expected != 0 ] && [ $nonzero_count_input == 0 ]; then
        verify_ret_value=$verify_ret_value:"All input values are zero.  Expected values are non-zero."
	zero_fail=1
    fi    

    # Loop through again to calculate differences and percentages
    for ((n=0;n<${#expected_values[@]};++n)); do
        procnum_exp=`echo "${expected_values[n]}" | cut -d " " -f1`
        run_time_exp=`echo "${expected_values[n]}" | cut -d " " -f2`
        idle_time_exp=`echo "${expected_values[n]}" | cut -d " " -f3`	

        procnum_inp=`echo "${input_data_array[n]}" | cut -d " " -f1`
        run_time_inp=`echo "${input_data_array[n]}" | cut -d " " -f2`
        idle_time_inp=`echo "${input_data_array[n]}" | cut -d " " -f3`
        
	# Make sure proc num matches.
        if [[ $procnum_exp != $procnum_inp ]]
	then
	    verify_ret_value=$verify_ret_value:"Mismatch processor number (expected DSP_$procnum_inp / acutal DSP_$procnum_inp)"
	fi

	# Check elapsed time
	if [ "$sample_window" != "" ] && [ "$elapsed_thresh" != "" ]; then
	    elapsed_diff=$(awk "BEGIN{x = (($run_time_inp + $idle_time_inp) - $sample_window); print x < 0 ? -x : x}")
	    result=$(awk "BEGIN{$elapsed_diff > $elapsed_thresh ? x=1 : x=0; print x}")
	    if [ $result -eq 1 ]; then
	        verify_ret_value=$verify_ret_value:"DSP_$procnum_exp run and idle time combined exceeds threshold for given sample window ($sample_window)."
	    fi	    
	fi 

	# Calculate differences.	
        run_diff=$(awk "BEGIN{x = ($run_time_exp - $run_time_inp); print x < 0 ? -x : x}")
	result=$(awk "BEGIN{$run_diff > $run_thresh ? x=1 : x=0; print x}")
        run_fail=0
	if [ $result -eq 1 ]; then
	    verify_ret_value=$verify_ret_value:"Run time for DSP_$procnum_exp exceeds threshold for expected value"
	    run_fail=1
	fi
	idle_diff=$(awk "BEGIN{x = ($idle_time_exp - $idle_time_inp); print x < 0 ? -x : x}")	
	result=$(awk "BEGIN{$idle_diff > $idle_thresh ? x=1 : x=0; print x}")
        idle_fail=0
	if [ $result -eq 1 ]; then
	    verify_ret_value=$verify_ret_value:"Idle time for DSP_$procnum_exp exceeds threshold for expected value"
	    idle_fail=1
	fi 
	
	# Calculate percentages.
	# Currently use percent diff with run 1
	run_percent=0
	idle_percent=0
	if [[ "$CURRENT_ITERATION" == 0 ]]; then
	    run_time_run1[n]=$run_time_inp
	    idle_time_run1[n]=$idle_time_inp
	else
	    run_diff_run1=$(awk "BEGIN{x = ("${run_time_run1[n]}" - $run_time_inp); print x < 0 ? -x : x}")
	    idle_diff_run1=$(awk "BEGIN{x = ("${idle_time_run1[n]}" - $idle_time_inp); print x < 0 ? -x : x}")
	    run_percent=$(awk "BEGIN{x = ($run_diff_run1 / "${run_time_run1[n]}"); x = (x * 100); print x < 0 ? -x : x}")
	    idle_percent=$(awk "BEGIN{x = ($idle_diff_run1 / "${idle_time_run1[n]}"); x = (x * 100); print x < 0 ? -x : x}")
if true; then
	    # check for divide by zero	
	    if [ $(awk "BEGIN{"${run_time_run1[n]}" == 0 ? x=1 : x=0; print x}") -eq 0 ]; then
	        run_percent=$(awk "BEGIN{x = ($run_diff_run1 / "${run_time_run1[n]}"); x = (x * 100); print x < 0 ? -x : x}")
	    else
		run_percent=100.0
		if [ $(awk "BEGIN{$run_time_exp == 0 ? x=1 : x=0; print x}") -eq 1 ]; then
		    run_percent=0.0
		fi
	    fi
	    
	    if [ $(awk "BEGIN{"${idle_time_run1[n]}" == 0 ? x=1 : x=0; print x}") -eq 0 ]; then
	    	idle_percent=$(awk "BEGIN{x = ($idle_diff_run1 / "${idle_time_run1[n]}"); x = (x * 100); print x < 0 ? -x : x}")
	    else
		idle_percent=100.0
		if [ $(awk "BEGIN{$idle_time_exp == 0 ? x=1 : x=0; print x}") -eq 1 ]; then
		    idle_percent=0.0
		fi
	    fi
	fi
fi
	# Print actual values on first time through to test summary.
	# For all else print percentages off of actual
	if [ $n == 0 ] && [ "$CURRENT_ITERATION" == 0 ]; then
            echo "#=============================================================================" >> $summary_file
	    if [[ "$ITERATIONS" -gt 1 ]]; then
		echo "Actual values for run 1:" >> $summary_file
	    else
		echo "Actual values:" >> $summary_file
	    fi
	    msg="Proc: "; printf "%-8s" $msg >> $summary_file
	    msg="run-time"; printf "%-12s" $msg >> $summary_file
	    msg=" idle-time"; printf "%-12s" $msg >> $summary_file
            msg=" run"; printf "%-5s" $msg >> $summary_file
            msg=" idle"; printf "%-5s\n" $msg >> $summary_file
	    echo "#=============================================================================" >> $summary_file
	fi

	if [[ "$CURRENT_ITERATION" == 0 ]]; then

            msg="DSP_${procnum_inp}: " ; printf "%-8s" $msg >> $summary_file
	    msg=" $run_time_inp"; printf "%-12s" $msg >> $summary_file
	    msg=" $idle_time_inp"; printf "%-12s" $msg >> $summary_file
	    if [[ $run_fail = 1 ]]; then
	        msg=" FAIL"; printf "%-5s" $msg >> $summary_file
	    else
	        msg=" PASS"; printf "%-5s" $msg >> $summary_file
	    fi
	    if [[ $idle_fail = 1 ]]; then
	        msg=" FAIL"; printf "%-5s\n" $msg >> $summary_file
	    else
	        msg=" PASS"; printf "%-5s\n" $msg >> $summary_file
	    fi
	else
	    if [[ $n == 0 ]]; then
		runnum=$((CURRENT_ITERATION + 1))
	        echo "#=============================================================================" >> $summary_file
	    	echo "Run $runnum data. Percent diff from run 1." >> $summary_file
		msg="Proc: "; printf "%-8s" $msg >> $summary_file
                msg=" run-time"; printf "%-12s" $msg >> $summary_file
                msg=" idle-time"; printf "%-12s" $msg >> $summary_file
                msg=" run"; printf "%-5s" $msg >> $summary_file
                msg=" idle"; printf "%-5s" $msg >> $summary_file
                msg=" run-pct"; printf "%-12s" $msg >> $summary_file
   	        msg=" idle-pct"; printf "%-12s\n" $msg >> $summary_file
	        echo "#=============================================================================" >> $summary_file
	    fi
            msg="DSP_${procnum_inp}: " ; printf "%-8s" $msg >> $summary_file
            msg=" $run_time_inp"; printf "%-12s" $msg >> $summary_file
            msg=" $idle_time_inp"; printf "%-12s" $msg >> $summary_file
	    if [[ $run_fail = 1 ]]; then
	        msg=" FAIL"; printf "%-5s" $msg >> $summary_file
	    else
	        msg=" PASS"; printf "%-5s" $msg >> $summary_file
	    fi
	    if [[ $idle_fail = 1 ]]; then
	        msg=" FAIL"; printf "%-5s" $msg >> $summary_file
	    else
	        msg=" PASS"; printf "%-5s" $msg >> $summary_file
	    fi
	    msg=" $run_percent"; printf "%-12s" $msg >> $summary_file
	    msg=" $idle_percent"; printf "%-12s\n" $msg >> $summary_file
	fi
    done
}

#----------------------------------------------------------------------
# Begin dsptop logging tests.
#
# logging mode/stop on full -l first
# quiet mode -q
# testmode 3 generates 1 of each message type.
# Note: if the trace buffer does not fill dsptop stalls forever.
#----------------------------------------------------------------------
rc_file=./.dsptoprc
if [ "$VERIFY" == "" ] && [ "$MATCH" == "" ]; then
logtest="first last"
example_path="/usr/share/ti/examples/opencl/mandelbrot"
example_name="mandelbrot"
for x in $logtest
do
    echo "logging test with $x option"
    logfile=${x}_log.txt
    loggingfailfile=${x}_logging_failures.txt
    rc_options_array="-q -l $x "
    rc_options_array+="-o $logfile"
    write_rc_file rc_options_array[@] $rc_file
    dsptop_sync "pushd $example_path && ./$example_name > /dev/null 2>&1 && popd"

    #validate log example
    loglist=(
        'Idle'
        'Running'
        'OpenMP'
        'OpenCL'
        'Device'
        'External'
        'Internal'
        'Exit'
        'Logging'
    )

    # Wait for $logfile to be closed
    while :
    do
        lsof | grep $logfile
        if [ $? -ne 0 ];
        then
           #echo "$logfile is not open"
           break
        #else
           #echo "$logfile is open"
      fi
    done

    total=$(cat $logfile | wc -l)
    found=0
    index=0

    while read -r line
    do
        ((index++))
        msg_recognized=0
        for i in ${loglist[@]}
        do
	    if [[ "$line" == *"$i"* ]]; then
	        msg_recognized=1
	    fi
        done
        if [[ $msg_recognized == 1 ]]; then
	    #echo "Log message recognized: $line"
	    ((found++))
        else
	    echo "Log message not recognized: $line"
	    echo ${index}: ${line} >> $loggingfailfile
        fi
    done <$logfile

    #echo "dsptop recognized $found logging instances out of $total lines"
    if [ $found = $total ]; then
        #echo "dsptop logging mode test PASS"
        TEST_SHORT_SUMMARY+=("dsptop_logging_${x} PASSED") 
    else
        #echo "dsptop logging mode test FAIL"
        TEST_SHORT_SUMMARY+=("dsptop_logging_${x} FAILED") 
        TEST_FAILURES+=("Error: Unknown output found in dsptop logging")
        echo "#=============================================================================" >> $failure_file 
        echo "#" >> $failure_file
        echo "#" >> $failure_file 
        echo "# Unrecognized lines for dsptop logging mode $x" >> $failure_file 
        echo "#" >> $failure_file
        echo "#" >> $failure_file
        echo "#=============================================================================" >> $failure_file
        echo "Found $found of $total lines" >> $failure_file
        cat $loggingfailfile >> $failure_file
    fi
done #end of logging test do loop
fi #end VERIFY/MATCH conditional

#---------------------------------------------------------------------
# Begin testing dsptop with mcsdk examples.
# Loop through config files and run tests
# Currently only testing with C66AK2Hxx
#----------------------------------------------------------------------
config_files="./$DEVICE/*"
logfile=./dsptop_log.txt

if [[ $ITERATIONS == "" ]]; then
    ITERATIONS=1
fi

for f in $config_files
do
    #echo "Processing $f"
    
    expected_values_array=()
    rc_options_array=()
    dontruntest=0
    
    if [[ $VERIFY != "" ]]; then
	if [[ $f != $VERIFY ]]; then
	    dontruntest=1
	fi
    fi

    if [[ $MATCH != "" ]]; then
	if [[ $f != $MATCH ]]; then
	    echo Skipping run and test of $f
	    continue
	fi
    fi

    # loop through each line of the file and parse entries
	
    elapsedtime_threshold=""

    while read p; do
	if [[ $p == "#"* ]]; then
	    # ignore commented lines in test config
	    continue
        elif [[ $p == "name:"* ]]; then
            example_name=`echo $p | cut -d : -f2`
	    #echo $example_name
        elif [[ $p == "pre_execute:"* ]]; then
            pre_execute=`echo $p | cut -d : -f2`
	    #echo $pre_execute
        elif [[ $p == "path:"* ]]; then
            example_path=`echo $p | cut -d : -f2`  
	    #echo $example_path
        elif [[ $p == "appargs:"* ]]; then
            appargs=`echo $p | cut -d : -f2`  
	    #echo $appargs
	elif [[ $p == "sample_window"* ]]; then
	    sample_window=`echo $p | cut -d : -f2`
	    rc_options_array+=("-s $sample_window")
	elif [[ $p == "delay"* ]]; then
	    delay=`echo $p | cut -d : -f2`
	    rc_options_array+=("-d $delay")
	elif [[ $p == "runtime_threshold:"* ]]; then
	    runtime_threshold=`echo $p | cut -d : -f2`
	elif [[ $p == "idletime_threshold:"* ]]; then
	    idletime_threshold=`echo $p | cut -d : -f2`
	elif [[ $p == "elapsedtime_threshold:"* ]]; then
	    elapsedtime_threshold=`echo $p | cut -d : -f2`
	elif [[ $p == "mem_exp:"* ]]; then
	    mem_exp=`echo $p | cut -d : -f2`
	elif [[ $p == "mem_thresh:"* ]]; then
	    mem_thresh=`echo $p | cut -d : -f2`
        elif [[ $p == "dsp "* ]]; then
	    # fill array with data points <procnum> <run_time> <idle_time>
	    pnum=`echo "$p" | cut -d " " -f2`
	    runt=`echo "$p" | cut -d " " -f4`
	    idlet=`echo "$p" | cut -d " " -f6`
	    expected_values_array[$pnum]="$pnum $runt $idlet" 
        fi
    done <$f
    
    if [[ $dontruntest == 1 ]]; then
	echo Skipping run and verify of $f
	continue
    fi
    echo "#=============================================================================" >> $summary_file 
    echo "#" >> $summary_file
    echo "#" >> $summary_file
    if [ "$ITERATIONS" -gt 1 ]; then
	echo "# Summary log for ${f} ($ITERATIONS iterations)" >> $summary_file 
    else
    	echo "# Summary log for $f" >> $summary_file 
    fi
    echo "#" >> $summary_file
    echo "#" >> $summary_file
    echo "#=============================================================================" >> $summary_file
    for ((iter=0; iter < $ITERATIONS; iter++)); do
	CURRENT_ITERATION=$iter
    	if [[ "$VERIFY" == "" ]]; then
	    rc_options_array+=("-p")
            write_rc_file rc_options_array[@] $rc_file
	    if [[ "$pre_execute" != "" ]]; then
	         dsptop_command="pushd $example_path && $pre_execute && ./$example_name $appargs  && popd"
	    else
    	        dsptop_command="pushd $example_path && ./$example_name $appargs > /dev/null 2>&1 && popd"
	    fi
    	    TI_TRACE_LOGGING=3 dsptop_sync "$dsptop_command" 
	fi
	# Print expected values on first time through to test summary.
	if [[ $iter == 0 ]]; then
	    echo "Expected values:" >> $summary_file
	    msg="Proc: "; printf "%-8s" $msg >> $summary_file
	    msg="run-time"; printf "%-12s" $msg >> $summary_file
	    msg=" idle-time"; printf "%-12s" $msg >> $summary_file
	    msg=" run-thresh"; printf "%-12s" $msg >> $summary_file
	    msg=" idle-thresh"; printf "%-12s\n" $msg >> $summary_file
	    echo "#=============================================================================" >> $summary_file
	    for ((m=0;m<${#expected_values_array[@]};++m)); do
        	procnum_exp=`echo "${expected_values_array[m]}" | cut -d " " -f1`
	        run_time_exp=`echo "${expected_values_array[m]}" | cut -d " " -f2`
        	idle_time_exp=`echo "${expected_values_array[m]}" | cut -d " " -f3` 
		msg="DSP_${procnum_exp}: " ; printf "%-8s" $msg >> $summary_file
		msg=" $run_time_exp"; printf "%-12s" $msg >> $summary_file
		msg=" $idle_time_exp"; printf "%-12s" $msg >> $summary_file
		msg=" $runtime_threshold"; printf "%-12s" $msg >> $summary_file
		msg=" $idletime_threshold"; printf "%-12s\n" $msg >> $summary_file
	    done
	fi
    	verify_single_log expected_values_array[@] $logfile $runtime_threshold $idletime_threshold $mem_exp $mem_thresh $sample_window $elapsedtime_threshold	
	
	IFS=":" read -ra error_array <<< "$verify_ret_value"
	# first entry is null error
    	if [[ ${#error_array[@]} -gt 1 ]]; then

	    #IFS=":" read -ra error_array <<< "$verify_ret_value"
	    # skip first element as it is blank
	    for ((err=1;err<${#error_array[@]};++err)); do
		if [ "$ITERATIONS" -gt 1 ]; then
		    TEST_FAILURES+=("${f}_$iter - ${error_array[err]}")
		else
		    TEST_FAILURES+=("$f - ${error_array[err]}")
		fi
	    done
	    
	    if [ "$ITERATIONS" -gt 1 ]; then
		TEST_SHORT_SUMMARY+=("${f}_$iter FAILED")
	    else
		TEST_SHORT_SUMMARY+=("$f FAILED")
	    fi
	    echo "#=============================================================================" >> $failure_file 
	    echo "#" >> $failure_file
	    echo "#" >> $failure_file
	    if [ "$ITERATIONS" -gt 1 ]; then
	         echo "# Failure log for ${f}_$iter" >> $failure_file 
	    else
	         echo "# Failure log for $f" >> $failure_file 
	    fi
	    echo "#" >> $failure_file
	    echo "#" >> $failure_file
	    echo "#=============================================================================" >> $failure_file
	    echo "# Errors:" >> $failure_file
	    echo "#=============================================================================" >> $failure_file
	    for ((err=1;err<${#error_array[@]};++err)); do
		echo "${err}. ${error_array[err]}" >> $failure_file 
	    done
	    echo "#=============================================================================" >> $failure_file 
	    echo "# Expected data values" >> $failure_file
	    echo "#=============================================================================" >> $failure_file
	    echo "External Mem Code/Data `echo $mem_exp | cut -d "," -f1` KB Total, `echo $mem_exp | cut -d "," -f2`% Used" >> $failure_file
    	    echo "External Mem Data Total `echo $mem_exp | cut -d "," -f3` KB Total, `echo $mem_exp | cut -d "," -f4`% Used" >> $failure_file
    	    echo "Internal Mem Data Total `echo $mem_exp | cut -d "," -f5` KB Total, `echo $mem_exp | cut -d "," -f6`% Used" >> $failure_file
	    echo " " >> $failure_file
	    echo -e "DSP_# \t\t RUN TIME \t\t IDLE TIME" >> $failure_file
	    for i in "${expected_values_array[@]}"
    	    do
	        pnum=`echo "$i" | cut -d " " -f1`
	        runt=`echo "$i" | cut -d " " -f2`
	        idlet=`echo "$i" | cut -d " " -f3`
                echo -e "DSP_$pnum \t\t $runt \t\t $idlet" >> $failure_file
    	    done
	    echo "#=============================================================================" >> $failure_file 
	    echo "# Threshold values" >> $failure_file
	    echo "#=============================================================================" >> $failure_file
    	    echo "Idle time threshold = $idletime_threshold" >> $failure_file
    	    echo "Run time threshold = $runtime_threshold" >> $failure_file
    	    echo "Elapsed time threshold = $elapsedtime_threshold" >> $failure_file
	    echo " " >> $failure_file
	    echo "External Mem Code/Data `echo $mem_thresh | cut -d "," -f1` KB Total, `echo $mem_thresh | cut -d "," -f2`% Used" >> $failure_file
    	    echo "External Mem Data `echo $mem_thresh | cut -d "," -f3` KB Total, `echo $mem_thresh | cut -d "," -f4`% Used" >> $failure_file
    	    echo "Internal Mem Data `echo $mem_thresh | cut -d "," -f5` KB Total, `echo $mem_thresh | cut -d "," -f2`% Used" >> $failure_file
            echo "#=============================================================================" >> $failure_file
	    echo "# Log data from test " >> $failure_file
	    echo "#=============================================================================" >> $failure_file
	    cat $logfile >> $failure_file
        else
	    if [ "$ITERATIONS" -gt 1 ]; then
		TEST_SHORT_SUMMARY+=("${f}_$iter PASSED")
	    else
		TEST_SHORT_SUMMARY+=("$f PASSED")
	    fi
        fi
    done
done

# Print testing summary
echo "#==============================================================================="
echo "# dsptop testing complete"
echo "#==============================================================================="
col=80
for i in "${TEST_SHORT_SUMMARY[@]}"
do
    msg=`echo "$i" | cut -d " " -f1`
    code=`echo "$i" | cut -d " " -f2`
    printf "%-58s" $msg 
    printf "%-6s\n" $code
done

if [ ${#TEST_FAILURES[@]} -eq 0 ]; then
    echo "#==============================================================================="
    echo "All dsptop tests passed"
    exit 0
else
    echo "#==============================================================================="
    echo "# ${#TEST_FAILURES[@]} error(s) detected when testing dsptop."
    echo "#==============================================================================="
    failindex=0
    for i in "${TEST_FAILURES[@]}"
    do
	((failindex++))
        echo "$failindex. $i"
    done
    echo "See $(pwd)/$failure_file for more information."
    exit 1
fi
