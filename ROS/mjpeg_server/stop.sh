#!/bin/sh

PIDFILE=/home/n800s/run/mjpeg_server.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
