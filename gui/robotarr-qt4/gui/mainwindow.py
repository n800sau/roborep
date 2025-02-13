# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""

from PyQt4.QtGui import QMainWindow, QGraphicsScene, QGraphicsLineItem, QPen, QColor
from PyQt4.QtCore import pyqtSignature, QTimer

from Ui_mainwindow import Ui_MainWindow

from lib.udprequest import UDPRequest

class MainWindow(QMainWindow, Ui_MainWindow):
	"""
	Class documentation goes here.
	"""
	def __init__(self, parent = None):
		"""
		Constructor
		"""
		QMainWindow.__init__(self, parent)
		self.setupUi(self)
		self.history = []
		self.W_Compass.direction_click.connect(self.onDirectionClick)
		for i in range(1, 6):
			w = getattr(self, 'W_Pwm%d' % i)
			w.value_change.connect(lambda value, wid=i: self.pwmSet(wid, value))
		self.timer = QTimer()
		self.timer.timeout.connect(self.onTimer)
		self.timer.start(1000)
		self.chn = UDPRequest('192.168.1.19', 9876)
	
	@pyqtSignature("int")
	def pwmSet(self, pwmid, pulse):
		self.chn.command('set_pwm_pulse', pwmid=pwmid-1, pulse=pulse)

	@pyqtSignature("")
	def onTimer(self):
		try:
			reply = self.chn.command('state')
			self.L_Version.setText(reply['version'])
			self.L_Battery.setText(str(reply['battery']))
			self.L_Pitch.setText(str(reply['pitch']))
			self.L_Roll.setText(str(reply['roll']))
			self.L_Orientation.setText('%d' % reply['heading'])
			self.L_Direction.setText('%d' % reply['current_heading'])
			self.W_Compass.setOrientation(int(reply['heading']))
			self.W_Compass.setDirection(int(reply['current_heading']))
			self.L_SpeedLeft.setText(str(reply['leftMotorSpeed']))
			self.L_SpeedRight.setText(str(reply['rightMotorSpeed']))
			self.L_Gx.setText(str(reply['Gx']))
			self.L_Gy.setText(str(reply['Gy']))
			self.L_Gz.setText(str(reply['Gz']))
			self.L_Lx.setText(str(reply['Lx']))
			self.L_Ly.setText(str(reply['Ly']))
			self.L_Lz.setText(str(reply['Lz']))
			for i in range(5):
				getattr(self, 'L_RawSensor%s' % i).setText(str(reply['ir_raw%s' % i]))
			start_index = self.history[-1].index if self.history else 0
			reply = self.chn.command('history', start_index=start_index+1)
#			print reply
		except:
			self.L_Error.setText('error')
	
	@pyqtSignature("")
	def on_TB_Forward_clicked(self):
		self.chn.command('accelerate')
	
	@pyqtSignature("")
	def on_TB_Left_clicked(self):
		self.chn.command('left')
	
	@pyqtSignature("")
	def on_TB_Stop_clicked(self):
		self.chn.command('stop')
	
	@pyqtSignature("")
	def on_TB_Right_clicked(self):
		self.chn.command('right')
	
	@pyqtSignature("")
	def on_TB_Back_clicked(self):
		self.chn.command('decelerate')

	@pyqtSignature("")
	def on_TB_Keep_clicked(self):
		self.chn.command('keep_direction')

	@pyqtSignature("")
	def on_TB_NoKeep_clicked(self):
		self.chn.command('stop_direction')
	
	@pyqtSignature("")
	def onDirectionClick(self):
		self.chn.command('set_direction', direction=self.W_Compass.direction())
		
