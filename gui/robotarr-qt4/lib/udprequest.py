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

	def command(self, cmd, **kwds):
		kwds['command'] = cmd
		data = json.dumps(kwds)
		self.sock.sendto(data, self.addr)
		reply,addr = self.sock.recvfrom(self.buf)
		return json.loads(reply)
