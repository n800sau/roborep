#!/usr/bin/env python

import socket, json, time
import urwid
from pprint import pformat

keylist = [
#'l3g4200d.js.obj',
#'hmc5883l.js.obj',
'bmp085.js.obj',
#'lsm303.js.obj',
'adxl345.js.obj',
#'mpu6050.js.obj',
#'kalman.js.obj',
#'mag3110.js.obj',
]

tab = '  '

def show_or_exit(key):
    if key in ('q', 'Q'):
        raise urwid.ExitMainLoop()
    txt.set_text(repr(key))

def on_idle():
	print idle

s = socket.create_connection(('115.70.59.149', 7980))
s.sendall(json.dumps({'cmd': 'send_full_data', 'interval': 100, 'count': 100}))
f = s.makefile()

def on_file():
	global lw
	dobj = json.loads(f.readline())
	ts_txt.set_text(dobj['s_timestamp'])
	dd = {}
	for k in keylist:
		if k in dobj:
			dd[k] = dobj[k]
	ll = []
	for k,v in dd.items():
		if isinstance(v, dict):
			ll.append(k)
			for vk,vv in v.items():
				if isinstance(vv, dict):
					ll.append("%s%s" % (tab, vk))
					for vvk,vvv in vv.items():
						ll.append("%s%s = %s" % (tab * 2, vvk, vvv))
				else:
					ll.append("%s%s = %s" % (tab, vk, vv))
		else:
			ll.append("%s = %s" % (k, v))
	json_txt.set_text("\n".join(ll))
	loop.draw_screen()

txt = urwid.Text(u"...")
ts_txt = urwid.Text(u"timestamp")
json_txt = urwid.Text(u"json")

pile = urwid.Pile([
	txt, 
	ts_txt,
	json_txt,
	])
p_fill = urwid.Filler(pile, 'top')

eloop = urwid.SelectEventLoop()
eloop.watch_file(f, on_file)


loop = urwid.MainLoop(p_fill,  unhandled_input=show_or_exit, event_loop=eloop)
loop.run()
