#!/bin/bash

cur_dir=`pwd`

python scareserver.py --conf="${cur_dir}/conf.json" &> ${cur_dir}/run.log &

echo $?
