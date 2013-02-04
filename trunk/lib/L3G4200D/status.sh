#!/bin/sh

PIDFILE=/home/n800s/run/l3g4200d.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/L3G4200D/l3g4200dd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
