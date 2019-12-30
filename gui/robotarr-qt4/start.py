#!/usr/bin/env python

from PyQt4.QtGui import QApplication
from gui.mainwindow import MainWindow
from lib.qt4_utils import qt4excepthook

if __name__ == "__main__":
	import sys
	sys.excepthook = qt4excepthook
	app = QApplication(sys.argv)
	mw = MainWindow()
	mw.show()
	sys.exit(app.exec_())

