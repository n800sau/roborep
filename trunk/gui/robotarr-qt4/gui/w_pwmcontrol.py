# -*- coding: utf-8 -*-

"""
Module implementing W_PwmControl.
"""

from PyQt4.QtGui import QWidget
from PyQt4.QtCore import pyqtSignature

from Ui_w_pwmcontrol import Ui_W_PwmControl

class W_PwmControl(QWidget, Ui_W_PwmControl):
    """
    Class documentation goes here.
    """
    def __init__(self, parent = None):
        """
        Constructor
        """
        QWidget.__init__(self, parent)
        self.setupUi(self)
    
    @pyqtSignature("int")
    def on_VS_Pulse_valueChanged(self, value):
        """
        Slot documentation goes here.
        """
        # TODO: not implemented yet
        raise NotImplementedError
