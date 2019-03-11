#!/bin/bash

cd `dirname $0`
#./motion2ftp.py &> motion2ftp.log
#./motion_files2ftp.py &> motion2ftp.log &
./motion_files2ftp_overwrite.py &> motion2ftp.log &
echo $?
