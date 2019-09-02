from pyfirmata import Arduino, util, boards

class Robo:

	def __init__(self):
		self.board = Arduino('/dev/ttyAMA0')

	def wakeup(self):
		#check time
	    print '!!!',self.board.get_firmata_version()
#		self.board.ser.write('get_current_time\x0a')
#		self.ser.read()
		#write log
		#get temperature
