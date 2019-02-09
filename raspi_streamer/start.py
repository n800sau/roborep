from __future__ import print_function
import os, sys, subprocess, locale, pwd, grp

from service import streamer

def application(env, start_response):

	for data in streamer.router(env, start_response):
		yield data
#	if rs is None:
#		start_response('404 Not Found', [('Content-Type','text/html')])
#		rs = [b'<pre>Environment: %s </pre>' % '\n'.join([('%s' % (item,)) for item in env.items()])]
#	return rs

