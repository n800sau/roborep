#!/bin/sh

FCGIPID="/home/n800s/www/robo.pid"

PID=`cat $FCGIPID`

if [ "x$PID" != "x" ]
then
	kill $PID && rm $FCGIPID
fi

