#!/usr/bin/env python

from flup.server.fcgi import WSGIServer
from flapp import app

if __name__ == '__main__':
    WSGIServer(app).run()
#    WSGIServer(app, bindAddress='/home/n800s/www/robo.sock').run()
#    WSGIServer(app, bindAddress=('localhost', 11111)).run()

