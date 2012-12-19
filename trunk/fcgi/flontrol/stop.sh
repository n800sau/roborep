#!/bin/sh

FCGIPID="/home/n800s/www/robo.pid"

PID=`cat $FCGIPID`

if [ "x$PID" != "x" ]
then
	sudo kill $PID && rm -f $FCGIPID
fi

