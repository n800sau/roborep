#!/usr/bin/env python

import sys, os

EVAL_PREFIX = "'python:"
INCLUDE_PREFIX = "'include:"

if len(sys.argv) < 3:
	print 'Usage: %s <input file> <output file>' % os.path.basename(__file__)
else:

	ifname = sys.argv[1]
	ofname = sys.argv[2]

	of = file(ofname, 'w')
	for l in file(ifname).readlines():
		of.write(l)
		if l.startswith(EVAL_PREFIX):
			of.write(eval(l[len(EVAL_PREFIX):].strip()))
		elif l.startswith(INCLUDE_PREFIX):
			of.write(file(l[len(INCLUDE_PREFIX):].strip()).read())
	of.close()
