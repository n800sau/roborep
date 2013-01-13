#!/bin/sh

PIDFILE=/home/n800s/run/camd.pid

start-stop-daemon --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
