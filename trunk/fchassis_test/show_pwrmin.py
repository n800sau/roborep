#!/usr/bin/env python

import sys, os, time, json

from lib.frobo_ng import frobo_ng
from lib.utils import dbprint

if __name__ == '__main__':


	c = frobo_ng()
	c.debug = True

	try:
		dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['lcount']))

		lpwr = c.find_left_minimum(True)
		dbprint('Min left: %d' % lpwr)
		c.wait_until_stop()
		dbprint('stopped=%s' % c.is_really_stopped())

		rpwr = c.find_right_minimum(True)
		dbprint('Min right: %d' % rpwr)
		c.wait_until_stop()
		dbprint('stopped=%s' % c.is_really_stopped())

		dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['lcount']))
	finally:
		c.cmd_mstop()
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
