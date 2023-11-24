#!/bin/bash

PID_LIST=`ps aux | grep espruino |grep -v grep| awk '{print $2}'`
echo $PID_LIST
if [[ ! -z "$PID_LIST" ]]
then
	kill $PID_LIST
fi

