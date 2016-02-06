#!/bin/bash

cur_dir=`pwd`

python "${cur_dir}/udprecv.py" &> ${cur_dir}/run.log &

echo $?
