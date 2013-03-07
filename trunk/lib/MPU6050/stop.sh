#!/bin/sh

PIDFILE=/home/n800s/run/mpu6050.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
