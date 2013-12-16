#!/bin/sh

PIDFILE=/home/n800s/run/freenect_server.pid
DAEMON="roslaunch"

PPID=`pidof -x -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
