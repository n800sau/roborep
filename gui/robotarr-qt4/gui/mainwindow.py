# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""

from PyQt4.QtGui import QMainWindow
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
		self.timer = QTimer()
		self.timer.timeout.connect(self.onTimer)
		self.timer.start(1000)
		self.chn = UDPRequest('192.168.1.19', 9876)
	
	@pyqtSignature("")
	def onTimer(self):
		try:
			reply = self.chn.command('version')
			self.L_Version.setText(reply)
			reply = self.chn.command('battery')
			self.L_Battery.setText(reply)
			reply = self.chn.command('orientation')
			pitch,roll,orientation = reply.split(':')
			self.L_Pitch.setText(pitch)
			self.L_Roll.setText(roll)
			self.L_Orientation.setText(orientation)
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
	
