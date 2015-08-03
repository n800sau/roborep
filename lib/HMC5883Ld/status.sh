#!/bin/sh

RUN_AS=n800s
PIDFILE=~/run/hmc5883l.pid
DAEMON=`pwd`/hmc5883ld

PPID=`pidof -o %PPID "${DAEMON}"`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
