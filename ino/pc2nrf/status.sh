#!/bin/sh

PIDFILE=${HOME}/run/nrf2tcp.pid
DAEMON="python"
unset PPID
if [ -e $PIDFILE ]
then
	PID=`cat $PIDFILE`
	PPID=`pidof -x -o %PPID ${DAEMON}|xargs -n1 echo|grep $PID`
fi
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as\n$PPID"
                     exit 0
             fi
