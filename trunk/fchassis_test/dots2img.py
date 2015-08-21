#!/usr/bin/env python

import os, json, math

from PIL import Image, ImageDraw

dots = json.load(file('dots.json'))
for k in dots.keys():
	x = 100
	y = 100
	xmin = 0
	xmax = 0
	ymin = 0
	ymax = 0
	for d in dots[k]:
		d['dX'] = math.sin(math.radians(d['heading']))
		d['dY'] = math.cos(math.radians(d['heading']))
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

im = Image.new('RGBA', (200, 200), (240, 240, 240, 0))
draw = ImageDraw.Draw(im)
colors = ('red', 'blue')
ci = 0
for k in dots.keys():
	clr = colors[ci]
	ci += 1 
	x = 100
	y = 100
	for d in dots[k]:
		draw.line((x, y, d['x'], d['y']), fill=clr)
		x = d['x']
		y = d['y']
im.save('/home/n800s/public_html/drawing.jpg')
