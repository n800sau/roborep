#!/bin/bash


source ~/.virtualenvs/gurus/bin/activate

cd ~/work/roborep/cv_test/garage

cur_dir=`pwd`

RUNNING=`ps -Af|grep run_sweep.py|grep python|grep -v grep|awk '{print $2}'`

if [ -z "$RUNNING" ]
then



/usr/bin/env python "${cur_dir}/run_sweep.py" &>> ${cur_dir}/run_sweep.log
echo $?

else

echo '###Found already running###'|tee -a ${cur_dir}/run_sweep.log

fi