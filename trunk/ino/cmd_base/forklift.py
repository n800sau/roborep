import time, traceback, sys, os, select, inspect, socket, cPickle, errno, collections
from multiprocessing import Process, Pipe, active_children

class ExceptionWrap(Exception):

	def __init__(self, v, ex, etrace):
		Exception.__init__(self, v)
		self.ex = ex
		self.etrace = etrace


class BSocket:

	def __init__(self, sock):
		self.ibuffer = ''
		self.sock = sock

	def need_read(self, timeout=0):
		return bool(select.select([self.sock], [], [], timeout)[0])

	def consume_readable(self, timeout=0):
		self.sock.setblocking(0)
		rs = []
		if self.need_read(timeout):
			while True:
				try:
					data = self.sock.recv(9000)
					if data:
						self.ibuffer += data
					else:
						break
				except socket.error, e:
					if e.errno == errno.EAGAIN:
						break
					else:
						raise

	def poll(self, timeout=0):
		while True:
			rs = self.ibuffer.find('\n\n')
			if rs >= 0:
				break
			self.consume_readable(timeout)
			if timeout == 0:
				break
		return rs

	def readline(self, timeout=0):
		pos = self.poll(timeout)
		if pos >= 0:
			rs = self.ibuffer[:pos]
			self.ibuffer = self.ibuffer[pos+2:]
		else:
			rs = ''
		return rs

	def recv_object(self, timeout=0):
		rs = None
		while True:
			objdata = self.readline(timeout)
			if objdata:
				rs = cPickle.loads(objdata)
				break
			elif not self.ibuffer:
				break
		return rs

	def send_object(self, data, timeout=0):
		self.sock.setblocking(0)
		data = cPickle.dumps(data) + '\n\n'
		msglen = len(data)
		totalsent = 0
		while totalsent < msglen:
			try:
				rl,wl,el = select.select([self.sock], [self.sock], [self.sock], timeout)
				if wl:
					sent = self.sock.send(data[totalsent:])
					if sent == 0:
						raise RuntimeError("socket connection broken")
					totalsent += sent
				if rl:
					self.consume_readable()
			except socket.error, e:
				if e.errno == errno.EAGAIN:
					self.consume_readable()
				else:
					raise

class SubProcessMethod:

	def __init__(self, interface, name):
		self.interface = interface
		self.name = name

	def __call__(self, *args, **kwds):
		return self.interface.callmethod(self.name, *args, **kwds)

class SubProcessInterface(object):
	"""master process class"""

	def __init__(self, process, evpipe_pair, robject):
		"""robject has to have methods:
			beginnotify
			endnotify
			exception
		"""
		self.process = process
		self.evpipe = evpipe_pair[0]
		self.bsock = BSocket(self.evpipe)
		#close subprocess pipe handle
		#it is needed in subprocess only
		evpipe_pair[1].close()
		self.robject = robject
		self.nextid = 1
		self.replylist = []
		self.callbackdict = {}

	def terminate(self):
		self.process.terminate()
		try:
			self.process.join()
		except OSError:
			pass

	def wait4reply(self, waitid):
		return self.process_events(waitid)

	def send_evdict(self, evdict):
		evdict['id'] = self.nextid
		self.nextid += 1
		self.bsock.send_object(evdict, 1)

	def send_message(self, message, *args, **kwds):
		self.send_evdict({'message': message, 'args': args, 'kwds': kwds})

	def callmethod(self, methodname, *args, **kwds):
		rs = None
		wait = kwds.pop('wait', False)
		if not wait:
			callback = kwds.pop('callback', None)
			if not callback is None:
				self.callbackdict[self.nextid] = (callback, kwds.pop('callback_id', None), kwds.pop('callback_arg', None))
		self.send_evdict({'method': methodname, 'args': args, 'kwds': kwds})
		if wait:
			rs = self.wait4reply(evdict['id'])
		return rs

	def __getattr__(self, name):
		return SubProcessMethod(self, name)

	def add2replylist(self):
		try:
			while True:
				reply = self.bsock.recv_object()
				if reply:
					self.replylist.append(reply)
				else:
					break
		except EOFError:
			if not self.process.is_alive():
				rsdone = True

	def is_alive(self):
		return self.process.is_alive()

	def process_events(self, waitid=None):
		rs = None
		rsdone = False
		while not rsdone:
			if waitid is None:
				#set rsdone to exit the loop because the loop is needed if waitid is not None
				rsdone = True
			self.add2replylist()
			while self.replylist:
				event = self.replylist.pop(0)
				if 'id' in event:
					if event['id'] in self.callbackdict.keys():
						rdata = event.get('return', None)
						cbdata = self.callbackdict.pop(event['id'])
						cbdata[0](rdata, callback_id=cbdata[1], callback_arg=cbdata[2])
					if (not waitid is None) and event['id'] == waitid:
						rs = event.get('return', None)
						#set rsdone to exit the outer loop
						rsdone = True
						break
				elif 'method' in event:
					if event['method'] == 'exception':
						raise ExceptionWrap('background process error', cPickle.loads(event['kwds']['exception']), cPickle.loads(event['kwds']['etraceback']))
					elif not self.robject is None:
						if hasattr(self.robject, event['method']):
							method = getattr(self.robject, event['method'])
							kwds = event.get('kwds', {})
							if 'interface' in inspect.getargspec(method).args:
								kwds['interface'] = self
							method(*event.get('args', []), **kwds)
						else:
							logmsg('No method %s found in the master' % event['method'])
		return rs

class SubProcess:
	"""slave process class"""

	def __init__(self, subobj, evpipe_pair):
		self.eventlist = []
		self.evpipe = evpipe_pair[1]
		self.bsock = BSocket(self.evpipe)
		self.subobj = subobj
		#assign self to the object attribute so it can use appeal_master_method
		#and now we have links to each other
		#objects become undeletable
		#that's ok
		#process termination should kill both
		self.subobj.subproc = self

	def send2master(self, data):
		self.bsock.send_object(data, timeout=5)

	def appeal_master_method(self, method, *args, **kwds):
		self.send2master({'method': method, 'args': args, 'kwds': kwds})

	def master_poll(self, timeout=None):
		request = self.bsock.recv_object(timeout)
		if request:
			self.eventlist.append(request)

	def master_says(self, timeout=0):
		rs = None
		self.master_poll(timeout=timeout)
		if self.eventlist:
			evdict = self.eventlist[0]
			if 'message' in evdict:
				rs = self.eventlist.pop(0)
		return rs

	def run(self):
		while True:
			self.processEvents()
			self. ... idle()

	def processEvents(self):
			try:
				self.master_poll()
				rs = bool(self.eventlist)
				while self.eventlist:
					event = self.eventlist.pop(0)
					if 'method' in event:
						if isinstance(getattr(self.subobj, event['method'], None), collections.Callable):
							self.appeal_master_method('beginnotify', methodname=event['method'])
							try:
								rs = getattr(self.subobj, event['method'])(*event.get('args', []), **event.get('kwds', {}))
								if 'id' in event:
									self.send2master({'id': event['id'], 'return': rs})
							finally:
								self.appeal_master_method('endnotify', methodname=event['method'])
						else:
							raise Exception('Method %s not found' % event['method'])
			except IOError, e:
				logerrormsg()
			except Exception, e:
				try:
					exc_type, exc_value, exc_traceback = sys.exc_info()
					self.appeal_master_method('exception', exception=cPickle.dumps(e), etraceback=cPickle.dumps(traceback.extract_tb(exc_traceback)))
				except:
					#the loop has not ever be finished from inside
					logerrormsg()
		return rs

def run_process(subprocess_obj, robject=None):
	evpipe_pair = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
	process = Process(target=SubProcess(subprocess_obj, evpipe_pair).run)
	process.daemon = False
	process.start()
	return SubProcessInterface(process, evpipe_pair, robject)

def terminate_all():
	for p in active_children():
		p.terminate()

