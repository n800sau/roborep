#!/bin/sh

PIDFILE=/home/n800s/run/userver.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/userver/userverd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
