#!/bin/bash

cd /home/n800s/work/roborep/ard_esp8266/IRproxy

cur_dir=`pwd`

#/usr/bin/python "${cur_dir}/udprecv.py" 2>&1 | rotatelogs -f ${cur_dir}/run.log 10M &
/usr/bin/python "${cur_dir}/udprecv.py" 2>&1 | logger -t irproxy &

echo $?
