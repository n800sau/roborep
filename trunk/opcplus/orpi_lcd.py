#!/usr/bin/env python

import time
from pyA20.gpio import gpio
from pyA20.gpio import port

class LCD16x2:

	# Define port to LCD mapping
	LCD_RS = port.PA21
	LCD_E  = port.PC3
	LCD_D4 = port.PA2
	LCD_D5 = port.PC7
	LCD_D6 = port.PC4
	LCD_D7 = port.PD14

	# Define some device constants
	LCD_WIDTH = 16    # Maximum characters per line
	LCD_CHR = True
	LCD_CMD = False

	LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
	LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

	# Timing constants
	E_PULSE = 0.0005
	E_DELAY = 0.0005

	def __init__(self):
		gpio.init()
		gpio.setcfg(self.LCD_E, gpio.OUTPUT)  # E
		gpio.setcfg(self.LCD_RS, gpio.OUTPUT) # RS
		gpio.setcfg(self.LCD_D4, gpio.OUTPUT) # DB4
		gpio.setcfg(self.LCD_D5, gpio.OUTPUT) # DB5
		gpio.setcfg(self.LCD_D6, gpio.OUTPUT) # DB6
		gpio.setcfg(self.LCD_D7, gpio.OUTPUT) # DB7
		self.lcd_init()

	def lcd_init(self):
		# Initialise display
		self.lcd_byte(0x33, self.LCD_CMD) # 110011 Initialise
		self.lcd_byte(0x32, self.LCD_CMD) # 110010 Initialise
		self.lcd_byte(0x06, self.LCD_CMD) # 000110 Cursor move direction
		self.lcd_byte(0x0C, self.LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
		self.lcd_byte(0x28, self.LCD_CMD) # 101000 Data length, number of lines, font size
		self.clear()
		time.sleep(self.E_DELAY)

	def lcd_byte(self, bits, mode):
		# Send byte to data pins
		# bits = data
		# mode = True  for character
		#        False for command

		gpio.output(self.LCD_RS, mode) # RS

		# High bits
		gpio.output(self.LCD_D4, False)
		gpio.output(self.LCD_D5, False)
		gpio.output(self.LCD_D6, False)
		gpio.output(self.LCD_D7, False)
		if bits&0x10==0x10:
			gpio.output(self.LCD_D4, True)
		if bits&0x20==0x20:
			gpio.output(self.LCD_D5, True)
		if bits&0x40==0x40:
			gpio.output(self.LCD_D6, True)
		if bits&0x80==0x80:
			gpio.output(self.LCD_D7, True)

		# Toggle 'Enable' pin
		self.lcd_toggle_enable()

		# Low bits
		gpio.output(self.LCD_D4, False)
		gpio.output(self.LCD_D5, False)
		gpio.output(self.LCD_D6, False)
		gpio.output(self.LCD_D7, False)
		if bits&0x01==0x01:
			gpio.output(self.LCD_D4, True)
		if bits&0x02==0x02:
			gpio.output(self.LCD_D5, True)
		if bits&0x04==0x04:
			gpio.output(self.LCD_D6, True)
		if bits&0x08==0x08:
			gpio.output(self.LCD_D7, True)

		# Toggle 'Enable' pin
		self.lcd_toggle_enable()

	def lcd_toggle_enable(self):
		# Toggle enable
		time.sleep(self.E_DELAY)
		gpio.output(self.LCD_E, True)
		time.sleep(self.E_PULSE)
		gpio.output(self.LCD_E, False)
		time.sleep(self.E_DELAY)

	def lcd_string(self, message,line):
		# Send string to display
		message = message.ljust(self.LCD_WIDTH, " ")

		self.lcd_byte(line, self.LCD_CMD)

		for i in range(self.LCD_WIDTH):
			self.lcd_byte(ord(message[i]), self.LCD_CHR)

	def clear(self):
		self.lcd_byte(0x01, self.LCD_CMD) # 000001 Clear display

if __name__ == '__main__':

	# Initialise display
	lcd = LCD16x2()

	try:
		while True:

			# Send some test
			lcd.lcd_string("Orange Pi", lcd.LCD_LINE_1)
			lcd.lcd_string("16x2 LCD Test", lcd.LCD_LINE_2)

			time.sleep(3) # 3 second delay

			# Send some text
			lcd.lcd_string("1234567890123456", lcd.LCD_LINE_1)
			lcd.lcd_string("abcdefghijklmnop", lcd.LCD_LINE_2)

			time.sleep(3) # 3 second delay

	finally:
		lcd.clear()
		lcd.lcd_string("Goodbye!", lcd.LCD_LINE_1)
