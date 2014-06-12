#!/bin/sh

PIDFILE=/home/n800s/run/pc2nrf.pid
DAEMON=/home/n800s/work/roborep/lib/pc2nrf/pc2nrfd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
