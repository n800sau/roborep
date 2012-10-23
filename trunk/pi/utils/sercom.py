import serial

class SerCom(serial.Serial):

	def eol_readline(self, eol='\n', endtexts=[]):
		rs = ''
		r = None
		while r!= eol and r!='' and rs not in endtexts:
			r = self.read(1)
			rs += r
		return rs

	def eol_readlines(self, eol='\n', mincount=0, endtexts=[], stoptexts=[':']):
		cnt = 0
		while True:
			rep = self.eol_readline(eol=eol, endtexts=endtexts + stoptexts)
			if not rep:
				if cnt > mincount:
					break
			else:
				cnt += 1
			yield rep
			if rep in stoptexts:
				break
