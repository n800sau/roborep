#!/usr/bin/env python

import sys, os, time, json

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':


	c = frobo_ng()
	c.debug = True

	for i in range(500):
		c.update_state()
		if c.state['sonar'] > 0:
#			print '%g(%g) m to %s (%g volts)' % (c.state['sonar'], c.state.get('irdist', -1), c.heading(), c.state['v'])
			break
		else:
			time.sleep(1)
	print '%g(%g) m to %s (%g volts)' % (c.state['sonar'], c.state.get('irdist', -1), c.heading(), c.state['v'])
