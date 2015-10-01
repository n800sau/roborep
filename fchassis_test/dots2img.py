#!/usr/bin/env python

import os, json, math

from PIL import Image, ImageDraw
import scipy.interpolate as sp
import numpy as np

dots = json.load(file('dots.json'))

kleft = "2"
kright = "3"

x = 0
y = 0
xmin = 0
xmax = 0
ymin = 0
ymax = 0
lcount = 0
rcount = 0
for d in dots:
	ldiff = d['lcount'] - lcount
	rdiff = d['rcount'] - rcount
	bdiff = (ldiff + rdiff) / 2.
	d['dX'] = math.sin(math.radians(d['heading'])) * bdiff
	d['dY'] = math.cos(math.radians(d['heading'])) * bdiff
	x += d['dX']
	y += d['dY']
	d['x'] = x
	d['y'] = y
	if x < xmin:
		xmin = x
	if x > xmax:
		xmax = x
	if y < ymin:
		ymin = y
	if y > ymax:
		ymax = y
print 'x=[%s:%s] y=[%s:%s]' % (xmin, xmax, ymin, ymax)
json.dump(dots, file('dots_processed.json', 'w'), indent=2)

xmin = max(xmin, -1000)
xmax = min(xmax, 1000)

ymin = max(ymin, -1000)
ymax = min(ymax, 1000)

im = Image.new('RGBA', (int(xmax-xmin), int(ymax-ymin)), (240, 240, 240, 0))
draw = ImageDraw.Draw(im)
ci = 0
zoom = 1
clr = 'blue'
x = (dots[0]['x'] - xmin) *zoom
y = (dots[0]['y'] - ymin) *zoom
draw.ellipse((x-5, y-5, x+5, y+5), fill=clr, outline=clr)
for d in dots[1:]:
	x1 = (d['x'] - xmin) * zoom
	y1 = (d['y'] - ymin) * zoom
	draw.line((x, y, x1, y1), fill=clr)
	x = x1
	y = y1
	a_x = d['acc']['x']
	a_y = d['acc']['y']
	draw.line((x, y, x + a_x, y + a_y), fill='green')
	if d['sonar'] > 0 and d['sonar'] < 0.2:
		draw.ellipse((x-2, y-2, x+2, y+2), fill='orange', outline='orange')
	if d.get('hit_warn', None):
		draw.ellipse((x-4, y-4, x+4, y+4), fill='yellow', outline='red')
im.save('/home/n800s/public_html/drawing.jpg')
