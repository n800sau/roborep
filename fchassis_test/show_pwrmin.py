#!/usr/bin/env python

import sys, os, time, json

from lib.frobo import frobo

if __name__ == '__main__':


	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
#		c.debug = False

		lpwr = c.find_left_minimum(False)
		print 'Min left: %d' % lpwr
		time.sleep(2)
		rpwr = c.find_right_minimum(False)
		print 'Min right: %d' % rpwr
		time.sleep(2)
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
