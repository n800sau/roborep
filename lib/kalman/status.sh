#!/bin/sh

PIDFILE=/home/n800s/run/kalman.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/kalman/kalmand

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
