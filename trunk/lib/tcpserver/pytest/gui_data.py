#!/usr/bin/env python

import socket, json, time
import urwid
from pprint import pformat

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
	dobj = json.loads(f.readline())
	ts_txt.set_text(dobj['s_timestamp'])
	json_txt.set_text(pformat(dobj))
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

