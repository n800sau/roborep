#!/usr/bin/env python

t = 350

p = 6

tot = 350
for i in range(30 * 12):
	add = tot * (p / 12.) / 100.
	tot = tot + add - 2.1

print 'tot=', tot
