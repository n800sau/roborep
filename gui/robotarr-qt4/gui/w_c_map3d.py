# -*- coding: utf-8 -*-

"""
Module implementing W_C_Map3D.
"""

from PyQt4.QtGui import QWidget
from PyQt4.QtCore import pyqtSignature

from Ui_w_c_map3d import Ui_W_C_Map3D

class W_C_Map3D(QWidget, Ui_W_C_Map3D):
	"""
	Class documentation goes here.
	"""
	def __init__(self, parent = None):
		"""
		Constructor
		"""
		QWidget.__init__(self, parent)
		self.setupUi(self)

	@pyqtSignature("")
	def on_TB_LeftF_clicked(self):
		self.W_Map3D.roboLeft(1)

	@pyqtSignature("")
	def on_TB_LeftB_clicked(self):
		self.W_Map3D.roboLeft(-1)

	@pyqtSignature("")
	def on_TB_RightF_clicked(self):
		self.W_Map3D.roboRight(1)

	@pyqtSignature("")
	def on_TB_RightB_clicked(self):
		self.W_Map3D.roboRight(-1)

	@pyqtSignature("")
	def on_TB_F_clicked(self):
		self.W_Map3D.roboF(1)

	@pyqtSignature("")
	def on_TB_Break_clicked(self):
		self.W_Map3D.roboStop()
