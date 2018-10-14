#!/bin/bash

cd `dirname $0`
./motion2ftp.py &> motion2ftp.log
echo $?
