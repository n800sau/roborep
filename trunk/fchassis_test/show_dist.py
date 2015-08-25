#!/usr/bin/env python

import sys, os, time, json

from lib.frobo import frobo

if __name__ == '__main__':


	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		c.debug = False

		while True:
			c.db_state()
			if c.curr_dist != [127] and c.curr_dist != 0:
				print '%s cm to %s' % (c.curr_dist, c.compass.heading())
				break
			else:
				time.sleep(1)
