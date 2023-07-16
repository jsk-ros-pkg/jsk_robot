#!/usr/bin/env bash

# Keep only the last 1000 lines of csv log before executing dstat
CSV_FILE=/home/fetch/Documents/jsk_dstat.csv

if [ -e $CSV_FILE ]; then
    cp -f $CSV_FILE $CSV_FILE.bk
    tail -n 1000 $CSV_FILE.bk > $CSV_FILE
    rm -f $CSV_FILE.bk
else
    touch $CSV_FILE
fi

dstat -tl --cpufreq -c -C all --top-cpu-adv -dgimnprsTy --output $CSV_FILE
