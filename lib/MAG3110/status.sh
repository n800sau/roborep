#!/bin/sh

PIDFILE=/home/n800s/run/mag3110.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/MAG3110/mag3110d

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
