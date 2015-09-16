#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
PIDFILE=~/run/l3g4200dd.pid
DAEMON=`pwd`/l3g4200dd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
