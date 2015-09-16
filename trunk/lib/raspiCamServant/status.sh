#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
PIDFILE=~/run/raspiCamServant.pid
DAEMON=`pwd`/raspiCamServantd

PPID=`pidof -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
