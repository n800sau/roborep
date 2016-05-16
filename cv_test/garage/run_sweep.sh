#!/bin/bash

cd /home/n800s/work/roborep/cv_test/garage

cur_dir=`pwd`

/usr/bin/env python "${cur_dir}/run_sweep.py" &>> ${cur_dir}/run_sweep.log
echo $?
