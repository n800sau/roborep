#!/bin/sh

PIDFILE=/home/n800s/run/camd.pid
PPID=`pidof -o %PPID camTr`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
