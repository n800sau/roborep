#!/bin/sh

PIDFILE=/home/n800s/run/oculus.pid
DAEMON=/home/n800s/work/robotarr-code/lib/OculusD/oculusd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
