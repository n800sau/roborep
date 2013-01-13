#!/bin/sh

PIDFILE=/home/n800s/run/camd.pid
if pidof -o %PPID camTr > /dev/null; then
                     echo "Running"
                     exit 0
             else
                     echo "Not running"
                     exit 1
             fi
