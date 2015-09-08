#!/usr/bin/env python

import sys, os, time, json

from lib.frobo import frobo
from lib.utils import dbprint

if __name__ == '__main__':


	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
#		c.debug = False

		try:
			dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.mleft['count'], c.mright['count']))
			lpwr = c.find_left_minimum(True)
			dbprint('Min left: %d' % lpwr)
			time.sleep(2)
			dbprint('stopped=%s' % c.is_really_stopped())
			rpwr = c.find_right_minimum(True)
			dbprint('Min right: %d' % rpwr)
			time.sleep(2)
			dbprint('stopped=%s' % c.is_really_stopped())
			dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.mleft['count'], c.mright['count']))
		finally:
			c.stop()
			time.sleep(1)
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
