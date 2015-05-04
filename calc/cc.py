#!/usr/bin/env python

t = 350

p = 6

load = 350000
tot = load
oldtot = load
pd = 0
mn = 2200
mrval = 270./7*30
y = 0
while tot > 0:
	mpd = 0
	mpdmr = 0
	for m in range(12):
		add = tot * (p / 12.) / 100.
		tot = tot + add - mn
		frac = mn - add
		print 'weekly percent payment=', add/30*7, 'principal payment', frac/30*7, 'total payment', mn/30*7
		mpd += mn
		mpdmr += mn - mrval
	pd += mpd
	tot += 200
	print 'y:%d, tot:%d, diff:%d, mpd:%d, mpdmr:%d, addp:%d' % (y,tot,oldtot-tot,mpd, mpdmr, (mn-mrval)/30.*7)
	oldtot = tot
	y += 1


print 'years=', y
