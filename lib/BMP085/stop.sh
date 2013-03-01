#!/bin/sh

PIDFILE=/home/n800s/run/bmp085.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
