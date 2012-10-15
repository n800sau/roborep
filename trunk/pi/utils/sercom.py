import serial

class SerCom(serial.Serial):

    def eol_readline(self, eol='\n'):
	rs = ''
	r = None
	while r!= eol and r!='':
	    r = self.read(1)
	    rs += r
	return rs

    def eol_readlines(self, eol='\n'):
	rs = []
	while True:
	    rep = self.eol_readline(eol='\r')
	    if not rep:
		break
	    rs.append(rep)
	return rs
