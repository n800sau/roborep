#!/bin/bash


source ~/.virtualenvs/gurus/bin/activate

cd ~/work/roborep/cv_test/garage

cur_dir=`pwd`

/usr/bin/env python "${cur_dir}/detect_label.py" 1>> ${cur_dir}/detect_label.log 2>> ${cur_dir}/detect_label_error.log
echo $?
