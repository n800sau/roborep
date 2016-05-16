#!/bin/bash

cd /home/n800s/work/roborep/ard_esp8266/IRproxy

cur_dir=`pwd`

/usr/bin/python "${cur_dir}/udprecv.py" &> ${cur_dir}/run.log &

echo $?
