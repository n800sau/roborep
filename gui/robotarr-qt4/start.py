#!/usr/bin/env python

from PyQt4.QtGui import QApplication
from gui.mainwindow import MainWindow

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())

