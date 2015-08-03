#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
PIDFILE=~/run/bmp085.pid
DAEMON=`pwd`/bmp085d

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
