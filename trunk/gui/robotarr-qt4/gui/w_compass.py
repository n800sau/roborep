from PyQt4.QtGui import QWidget, QPainter, QPen, QPalette, QColor, QBrush
from PyQt4.QtCore import Qt, QLine, QPoint, QRect, QSize, QTimer

class W_Compass(QWidget):

	def __init__(self, parent):
		QWidget.__init__(self, parent)
		self.__orientation__ = None

	def setOrientation(self, deg):
		self.__orientation__ = deg
		self.update()

	def orientation(self):
		return self.__orientation__

	def paintEvent(self, event):
		if not self.__orientation__ is None:
			painter = QPainter(self)
			painter.save();
			painter.setRenderHint(QPainter.Antialiasing)
			r = self.rect().adjusted(10, 10, -10, -10)
			minside = min(r.width(), r.height())
			r = QRect((self.rect().width() - minside)/2, (self.rect().height() - minside)/2, minside, minside)
			pen = QPen(QColor('pink'))
			pen.setWidth(5)
			painter.setPen(pen)
			painter.drawEllipse(r);
			painter.translate(self.rect().center());
			for d in ('N', 'E', 'S', 'W'):
				painter.drawLine(0, r.height()/2, 0, r.height()/2 - 10)
				painter.drawText(0, r.height()/2, d)
				painter.rotate(90.0)
			painter.rotate(self.__orientation__);
			pen = QPen(QColor('blue'))
			pen.setWidth(2)
			painter.setPen(pen)
			painter.drawPolygon(QPoint(-10, -10), QPoint(10, -10), QPoint(0, - r.width()/2 + 10))
			painter.restore();
