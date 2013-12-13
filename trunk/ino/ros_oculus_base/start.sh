#!/bin/sh

PIDFILE=${HOME}/run/oculus_mega.pid
DAEMON="`which rosrun` rosserial_python serial_node.py /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A4015M29-if00-port0"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
