#!/bin/sh

FCGIPID="/home/n800s/www/bon.pid"

PID=`cat $FCGIPID`

if [ "x$PID" != "x" ]
then
	kill $PID && rm $FCGIPID
fi

