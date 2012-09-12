#!/usr/bin/env python

import sys, os

EVAL_PREFIX = "'python:"
INCLUDE_PREFIX = "'include:"

if len(sys.argv) < 3:
	print 'Usage: %s <input file> <output file>' % os.path.basename(__file__)
else:

	def preprocess(inplines):
		rs = []
		for l in inplines:
			l = l.rstrip()
			rs.append(l)
			if l.startswith(EVAL_PREFIX):
				rs += eval(l[len(EVAL_PREFIX):].strip()).split('\n')
			elif l.startswith(INCLUDE_PREFIX):
				olines = preprocess(file(l[len(INCLUDE_PREFIX):].strip()).readlines())
				rs += olines
		return rs

	ifname = sys.argv[1]
	ofname = sys.argv[2]

	outlines = preprocess(file(ifname).readlines())

	of = file(ofname, 'w')
	of.write('\n'.join(outlines))
	of.close()
