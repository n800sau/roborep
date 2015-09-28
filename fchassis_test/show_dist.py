#!/usr/bin/env python

import sys, os, time, json

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':


	c = frobo_ng()
#	c.debug = True

	for i in range(5):
		c.update_state()
		if c.state['sonar'] > 0:
			break
		else:
			time.sleep(1)
	print '%d cm to %s' % (c.state['sonar'] * 100, c.compass.heading())
