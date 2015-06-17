#!/usr/bin/python
import sys, os

#sys.stderr = file(os.path.join(os.path.dirname(__file__), 'fcgi.log'), 'w')

from flup.server.fcgi import WSGIServer
from app import app
import logging
from logging.handlers import RotatingFileHandler

if __name__ == '__main__':
#	handler = RotatingFileHandler('app.log', maxBytes=10000, backupCount=1)
#	handler.setLevel(logging.INFO)
#	app.logger.addHandler(handler)
	app.debug = True
	WSGIServer(app, bindAddress=('localhost', 5000)).run()
#WSGIServer(app, bindAddress='/tmp/app-fcgi.sock').run()
#WSGIServer(app).run()
