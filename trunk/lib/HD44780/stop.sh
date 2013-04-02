#!/bin/sh

PIDFILE=/home/n800s/run/hd44780.pid

start-stop-daemon -v --stop --user root --pidfile ${PIDFILE} --signal 15 --retry 10
