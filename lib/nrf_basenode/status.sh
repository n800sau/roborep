#!/bin/sh

PIDFILE=/home/n800s/run/nrf_basenode.pid
DAEMON=/home/n800s/work/robotarr-code/lib/nrf_basenode/nrf_basenoded

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
