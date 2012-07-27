#!/usr/bin/env python

import sys
from PyQt4.QtGui import QApplication, QMainWindow
from mainwindow import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = MainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
