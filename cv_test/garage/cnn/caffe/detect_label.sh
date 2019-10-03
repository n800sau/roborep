#!/bin/bash


export GLOG_minloglevel=3

cd ~/work/roborep/cv_test/garage/cnn/caffe

cur_dir=`pwd`

RUNNING=`ps -Af|grep detect_label.py|grep python|grep -v grep|awk '{print $2}'`

if [ -z "$RUNNING" ]
then

/usr/bin/env python -u "${cur_dir}/detect_label.py" --arch data/garage_deploy.prototxt --classlist data/class.lst \
    --snapshot data/snapshots/garage_iter_5000.caffemodel.h5 \
    --mean data/dataset_mean.binaryproto 1>> ${cur_dir}/detect_label.log 2>> ${cur_dir}/detect_label_error.log
echo $?

else

echo `date --rfc-3339=seconds`'###Found already running###'|tee -a ${cur_dir}/detect_label_error.log

fi
