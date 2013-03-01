#!/bin/sh

PIDFILE=/home/n800s/run/bmp085.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/BMP085/bmp085d

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
