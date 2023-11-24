# -*- coding: utf-8 -*-

"""
Module implementing W_PwmControl.
"""

from PyQt4.QtGui import QWidget
from PyQt4.QtCore import pyqtSignature

from Ui_w_pwmcontrol import Ui_W_PwmControl

class W_PwmControl(QWidget, Ui_W_PwmControl):
	
	value_change = pyqtSignal('int')

	def __init__(self, parent = None):
		QWidget.__init__(self, parent)
		self.setupUi(self)
	
	@pyqtSignature("int")
	def on_VS_Pulse_valueChanged(self, value):
		self.value_change.emit(value + 1500)
