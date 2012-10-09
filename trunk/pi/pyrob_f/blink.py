#!/usr/bin/env python

import time
import pyfirmata
import reset

reset.reset_m()

PIN = 12 # Pin 12 is used

# Adjust that the port match your system, see samples below:
# On Linux: /dev/tty.usbserial-A6008rIF, /dev/ttyACM0, 
# On Windows: \\.\COM1, \\.\COM2
PORT = '/dev/ttyAMA0'

# Creates a new board 
board = pyfirmata.Arduino(PORT)

# Loop for blinking the led
#while True:
print 'begin'
board.digital[PIN].write(0) # Set the LED pin to 1 (HIGH)
board.pass_time(1)
board.digital[PIN].write(1) # Set the LED pin to 1 (HIGH)
print 'now'
board.pass_time(2)
board.digital[PIN].write(0) # Set the LED pin to 0 (LOW)
#board.pass_time(DELAY)
print 'end'

