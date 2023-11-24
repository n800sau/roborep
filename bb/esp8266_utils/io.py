#!/usr/bin/env python3

#config-pin P9_14 gpio

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
import serial, time, sys

#print(UART.__file__)

#P8_13 -> i00
#P8_14 -> chip_en
#UART2 - TX - P9_21
#        RX - P9_22

# Set up pins as inputs or output
GPIO.setup("P8_13", GPIO.OUT)
GPIO.setup("P8_14", GPIO.OUT)
#GPIO.setup("GPIO0_26", GPIO.OUT)  # Alternative: use actual pin names

UART.setup("UART2")

# programming
def program():
    GPIO.output("P8_13", GPIO.LOW)  # You can also write '1' instead
    GPIO.output("P8_14", GPIO.LOW)   # You can also write '0' instead
    time.sleep(0.1)
    GPIO.output("P8_14", GPIO.HIGH)   # You can also write '0' instead

# working
def work():
    GPIO.output("P8_13", GPIO.HIGH)  # You can also write '1' instead
    GPIO.output("P8_14", GPIO.LOW)   # You can also write '0' instead
    time.sleep(0.1)
    GPIO.output("P8_14", GPIO.HIGH)   # You can also write '0' instead


work()
#program()

sys.exit()

def print_serial():
	with serial.Serial(port = "/dev/ttyO2", baudrate=74880) as ser:
#	with serial.Serial(port = "/dev/ttyO2", baudrate=115200) as ser:
		while True:
			print(ser.readline().decode('utf-8', 'replace'))


print_serial()
sys.exit()



