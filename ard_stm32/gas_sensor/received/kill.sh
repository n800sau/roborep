#!/bin/sh
PID=`ps -eo pid,command | grep recv.py  | grep -v grep | awk '{print $1}'`
kill $PID
echo $?