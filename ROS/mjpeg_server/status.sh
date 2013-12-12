#!/bin/sh

PIDFILE=/home/n800s/run/mjpeg_server.pid
DAEMON="mjpeg_server mjpeg_server _port:=1935"

PPID=`pidof -x -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
