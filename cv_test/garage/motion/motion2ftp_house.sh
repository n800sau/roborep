#!/bin/bash

cd `dirname $0`
python -u motion_files2ftp_overwrite.py house &> motion2ftp.log &
echo $?