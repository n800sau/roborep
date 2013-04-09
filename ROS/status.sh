#!/bin/sh

PIDFILE=/home/n800s/run/roscore.pid
DAEMON=`which roscore`

PPID=`pidof -x -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
