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
		self.W_Compass.direction_click.connect(self.onDirectionClick)
		self.timer = QTimer()
		self.timer.timeout.connect(self.onTimer)
		self.timer.start(10)
		self.chn = UDPRequest('192.168.1.19', 9876)
	
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
		