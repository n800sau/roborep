#!/bin/bash
cd /home/n800s/work/roborep/ard_stm32/gas_sensor/received
date > last_time_started.txt
./enable-spi.sh
./run.sh &
