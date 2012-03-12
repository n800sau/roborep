from socket import *
import json

class UDPRequest:

	def __init__(self, host, port):
		self.buf = 1024
		self.addr = (host, port)
		self.sock = socket(AF_INET,SOCK_DGRAM)
		self.sock.settimeout(1)

	def __del__(self):
		self.sock.close()

	def command(self, cmd, *args, **kwds):
		if args:
			cmd += ':' + ','.join([str(arg) for arg in args]) 
		if kwds:
			cmd += ':' + ','.join(['%s=%s' % (n,v) for n,v in kwds.items()])
		cmd = json.dumps({'cmd': cmd, 'args': args, 'kwds': kwds})
		self.sock.sendto(cmd, self.addr)
		reply,addr = self.sock.recvfrom(self.buf)
		return reply
