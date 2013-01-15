#!/bin/sh

PIDFILE=/home/n800s/run/hmc5883l.pid
DAEMON=/home/n800s/work/sourceforge/robotarr-code/lib/HMC5883Ld/hmc5883ld

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
