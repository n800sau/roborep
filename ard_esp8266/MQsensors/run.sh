#!/bin/bash

cd `dirname $0`

cur_dir=`pwd`

#/usr/bin/env python "${cur_dir}/udp_recv.py" 2>&1 | rotatelogs -f ${cur_dir}/run.log 10M &
/usr/bin/env python "${cur_dir}/udp_recv.py" 2>&1 | logger -t mqsensor &

echo $?
