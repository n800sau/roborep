#!/bin/bash

cd `dirname $0`
cur_dir=`pwd`

/usr/bin/env python "${cur_dir}/gate_speak.py" &>> ${cur_dir}/gate_speak.log
echo $?
