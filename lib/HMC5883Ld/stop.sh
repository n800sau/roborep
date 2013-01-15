#!/bin/sh

PIDFILE=/home/n800s/run/hmc5883l.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
