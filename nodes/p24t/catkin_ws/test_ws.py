#!/usr/bin/env python


#URI = "ws://echo.websocket.org/"
URI = 'ws://localhost:9090'
URI = 'ws://n800s.ddns.net:33290'

# 9089
#URI = 'ws://n800s.ddns.net:33289'

import websocket
try:
	import thread
except ImportError:
	import _thread as thread
import time

def on_message(ws, message):
	print(message)

def on_error(ws, error):
	print(error)

def on_close(ws):
	print("### closed ###")

def on_open(ws):
	def run(*args):
		for i in range(3):
			time.sleep(1)
			ws.send("Hello %d" % i)
		time.sleep(1)
		ws.close()
		print("thread terminating...")
	thread.start_new_thread(run, ())


if __name__ == "__main__":
	websocket.enableTrace(True)
	ws = websocket.WebSocketApp(URI, on_message = on_message, on_error = on_error, on_close = on_close)
	ws.on_open = on_open
	ws.run_forever()
