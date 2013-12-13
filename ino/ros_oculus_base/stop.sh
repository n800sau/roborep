#!/bin/sh

PIDFILE=${HOME}/run/oculus_mega.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
