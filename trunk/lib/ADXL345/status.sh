#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
PIDFILE=~/run/adxl345.pid
DAEMON="`pwd`/adxl345d"

PPID=`pidof -o %PPID "${DAEMON}"`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
