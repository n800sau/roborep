import sys, traceback
from PyQt4.QtGui import QMessageBox, QApplication
from PyQt4.QtCore import QCoreApplication

def qt4excepthook(excType, excValue, tracebackobj):
	traceback.print_exception(excType, excValue, tracebackobj)
	app = QCoreApplication.instance()
	if not app:
		app = QApplication(sys.argv)
	QMessageBox.critical(None, 'Critical Error', u'\n'.join(traceback.format_exception_only(excType, excValue)))
