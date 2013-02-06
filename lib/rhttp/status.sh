#!/bin/sh

PIDFILE=/home/n800s/run/rhttp.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/ADXL345/rhttpd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
