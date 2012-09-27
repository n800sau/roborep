from pyfirmata import Arduino, util, boards

class Robo:

	def __init__(self):
		self.board = Arduino('/dev/ttyATA0')

	def wakeup(self):
		#check time
		self.board.ser.write('get_current_time\x0a')
		self.ser.read()
		#write log
		#get temperature
