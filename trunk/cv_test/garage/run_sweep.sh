#!/bin/bash


source ~/.virtualenvs/gurus/bin/activate

cd ~/work/roborep/cv_test/garage

cur_dir=`pwd`

/usr/bin/env python "${cur_dir}/run_sweep.py" &>> ${cur_dir}/run_sweep.log
echo $?
