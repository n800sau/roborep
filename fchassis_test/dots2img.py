#!/usr/bin/env python

import os, json, math

from PIL import Image, ImageDraw
import scipy.interpolate as sp
import numpy as np

dots = json.load(file('dots.json'))

kleft = "2"
kright = "3"

for k in dots.keys():
	x = 100
	y = 100
	xmin = 0
	xmax = 0
	ymin = 0
	ymax = 0
	if dots[k]:
		d = dots[k][0]
		x += (-1 if k == kleft else 1) * math.sin(math.radians(d['heading']))
		y += (-1 if k == kleft else 1) * 20 * math.cos(math.radians(d['heading']))
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


im = Image.new('RGBA', (800, 800), (240, 240, 240, 0))
draw = ImageDraw.Draw(im)
colors = ('red', 'blue')
ci = 0
zoom = 4
for k in dots.keys():
	if dots[k]:
		clr = colors[ci]
		ci += 1 
		x = dots[k][0]['x']*zoom
		y = dots[k][0]['y']*zoom
		draw.ellipse((x-5, y-5, x+5, y+5), fill=clr, outline=clr)
		for d in dots[k][1:]:
			draw.line((x, y, d['x']*zoom, d['y']*zoom), fill=clr)
			x = d['x']*zoom
			y = d['y']*zoom
			a_x = d['acc']['x']
			a_y = d['acc']['y']
			draw.line((x, y, x + a_x, y + a_y), fill='green')
			if d['dist'] > 0 and d['dist'] < 20:
				draw.ellipse((x-2, y-2, x+2, y+2), fill='orange', outline='orange')
			if d.get('hit_warn', None):
				draw.ellipse((x-4, y-4, x+4, y+4), fill='yellow', outline='red')
im.save('/home/n800s/public_html/drawing.jpg')
