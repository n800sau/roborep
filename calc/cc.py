#!/usr/bin/env python

t = 350

p = 5.3

tot = 350
for i in range(30):
	tot = tot * (1 + p / 100.) - 12 * 2

print 'tot=', tot
