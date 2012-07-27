# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""

import random, time
from datetime import datetime, timedelta

from PyQt4.QtGui import QMainWindow, QColor
from PyQt4.QtCore import pyqtSignature, QTimer, Qt

from Ui_mainwindow import Ui_MainWindow

from PyQt4.Qwt5 import QwtPlotCurve, QwtPlotItem, QwtScaleDraw, QwtPlot, QwtText
from PyQt4.Qwt5.anynumpy import arange, sin

class TimeScaleDraw(QwtScaleDraw):

	def __init__(self, timestamps, *args):
		QwtScaleDraw.__init__(self, *args)
		self.timestamps = timestamps

	def label(self, value):
		return QwtText(time.strftime("%H:%M:%S", time.localtime(self.timestamps[int(value)]))  if value < len(self.timestamps) else '')

class StatCurve(QwtPlotCurve):

	def __init__(self, *args):
		QwtPlotCurve.__init__(self, *args)
		self.setRenderHint(QwtPlotItem.RenderAntialiased)

	def setColor(self, color):
		c = QColor(color)
		c.setAlpha(150)
		self.setPen(c)
		self.setBrush(c)

HISTORY = 60

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
		self.data = []
		self.timestamps = []
		self.timer = QTimer()
		self.timer.timeout.connect(self.onTimeOut)
		self.dataTimer = QTimer()
		self.dataTimer.timeout.connect(self.onDataTimeOut)
		self.dataTimer.start(100)
		self.qP_Time.setAxisScaleDraw(QwtPlot.xBottom, TimeScaleDraw(self.timestamps))
		self.qP_Time.setAxisScale(QwtPlot.xBottom, 0, HISTORY)
		self.qP_Time.setAxisLabelRotation(QwtPlot.xBottom, -50.0)
		self.qP_Time.setAxisLabelAlignment(QwtPlot.xBottom, Qt.AlignLeft | Qt.AlignBottom)
		self.qP_Time.setAxisScale(QwtPlot.yLeft, -100, 100)
		self.curve = StatCurve('System')
		self.curve.setData(range(HISTORY), self.data)
		self.curve.attach(self.qP_Time)
		self.curve.setVisible(True)
	
	@pyqtSignature("")
	def onTimeOut(self):
		self.curve.setData(range(HISTORY), self.data)
		self.qP_Time.replot()

	@pyqtSignature("")
	def on_PB_Start_clicked(self):
		self.timer.start(100)
	
	@pyqtSignature("")
	def on_PB_Stop_clicked(self):
		self.timer.stop()

	@pyqtSignature("")
	def onDataTimeOut(self):
		t = time.time() + (random.random() * 0.050 - 0.025)
		self.timestamps.append(t)
		self.timestamps = self.timestamps[-HISTORY:]
		self.data.append((sin(self.timestamps[-1]) + random.random() - 0.5) * 50)
		self.data = self.data[-HISTORY:]
		print self.timestamps[-1], self.data[-1]
