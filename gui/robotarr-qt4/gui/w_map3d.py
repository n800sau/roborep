import math, time, array
from PyQt4.QtGui import QWidget, QPainter, QPen, QPalette, QColor, QBrush, QGraphicsView, QGraphicsItem, QGraphicsScene
from PyQt4.QtCore import Qt, QLine, QPoint, QPointF, QRectF, QRect, QSize, QTimer, pyqtSignal

class Robo:

	def __init__(self):
		self.direction = 0

	def paint(self, painter, rect):
		painter.save()
		try:
			painter.translate(rect.center())
			painter.rotate(180 + self.direction)
			clr = QColor('blue')
			pen = QPen(clr)
			pen.setWidth(0)
			painter.setPen(pen)
			w = rect.width()
			h = rect.height()
			r = QRectF(-w / 2., -h / 2., w, h)
			painter.drawRect(r)
			p = QPointF(0, h)
			painter.drawLine(p, r.bottomLeft())
			painter.drawLine(p, r.bottomRight())
		finally:
			painter.restore()

class BlackBox:

	def __init__(self):
		pass

	def paint(self, painter, rect):
		painter.fillRect(rect, QColor('black'))

class World:

	#size of a unit - 1 cm 0,0,0 - in the north west corner
	def __init__(self, xs, ys, zs, robox, roboy, roboz):
		self.xsize = xs
		self.ysize = ys
		self.zsize = zs
		self.matrix = array.array('h', [-1] * xs * ys * zs)
		self.objects = []
		self.scale = 1
		self.robo = Robo()
		self.setRoboLocation(robox, roboy, roboz)

	def rect(self):
		return QRect(0, 0, self.xsize * self.scale, self.ysize * self.scale)

	def setRoboLocation(self, x, y, z):
		self.roboloc = (x, y, z)

	def posidx(self, x, y, z):
		return x + y * self.xsize + z * self.xsize * self.ysize

	def setScale(self, scale):
		self.scale = scale

	def found(self, o, x, y, z):
		pos = self.posidx(x, y, z)
		idx = self.matrix[pos]
		if idx < 0:
			idx = len(self.objects)
			self.objects.append(o)
			self.matrix[pos] = idx
		else:
			self.objects[idx] = o

	def vanished(self, x, y, z):
		pos = self.posidx(x, y, z)
		self.matrix[pos] = -1

	def paint(self, painter, slice_z, ruler_x, ruler_y):
		ruler_step = max(1, 50 // self.scale)
		painter.fillRect(self.rect(), QColor('white'))
		pen = QPen(QColor('green'))
		pen.setWidth(0)
		painter.setPen(pen)
		#draw objects
		for y in range(self.ysize):
			for x in range(self.xsize):
				idx = self.matrix[slice_z * self.ysize * self.xsize + y * self.xsize + x]
				if idx >= 0:
					o = self.objects[idx]
					if o:
						r = QRectF(x * self.scale, y * self.scale, self.scale, self.scale)
						o.paint(painter, r)
				# draw ruler
				if (x - ruler_step // 2) % ruler_step == 0:
					painter.drawLine(QPointF(x*self.scale, ruler_y - 3), QPointF(x*self.scale, ruler_y))
					painter.drawText(QPointF(x*self.scale, ruler_y - 20), str(x))
			# draw ruler
			if (y - ruler_step // 2) % ruler_step == 0:
				painter.drawLine(QPointF(ruler_x, y*self.scale), QPointF(ruler_x + 3, y*self.scale))
				painter.drawText(QPointF(ruler_x + 3, y*self.scale), str(y))
		# draw robo
		self.robo.paint(painter, QRectF(self.roboloc[0] * self.scale, self.roboloc[1] * self.scale, self.scale, self.scale))

class W_Map3D(QWidget):

	def __init__(self, parent=None):
		QWidget.__init__(self, parent)
		self.world = World(100, 100, 100, 50, 50, 0)
		self.world.found(BlackBox(), 10, 20, 0)
		self.dx = 0
		self.dy = 0
		self.slice_z = 0

	def paintEvent(self, event):
		painter = QPainter(self)
		painter.save()
		try:
			painter.setRenderHint(QPainter.Antialiasing)
			r = self.rect().adjusted(10, 10, -10, -10)
			wr = self.world.rect()
			xoff = r.center().x() - wr.center().x() + self.dx
			yoff = r.center().y() - wr.center().y() + self.dy
			painter.translate(xoff, yoff)
			self.world.paint(painter, self.slice_z, r.left() - xoff, r.bottom() - yoff)
		finally:
			painter.restore();

	def scaleWorld(self, scale):
		if scale > 0:
			self.world.setScale(scale)
			self.update()

	def moveWorld(self, dx, dy, dz):
		self.slice_z += dz

	def wheelEvent(self, event):
		numDegrees =  - (event.delta() / 16.)
		d_scale = (1 + numDegrees/100.)  if numDegrees > 0 else (1 + numDegrees/100.)
		self.scaleWorld(self.world.scale * (d_scale if d_scale> 0 else 1/-d_scale))

	def mousePressEvent(self, event):
		self.d_old = (self.dx, self.dy)
		self.m_pos = event.pos()

	def mouseMoveEvent(self, event):
		diff_pos = event.pos() - self.m_pos
		self.dx, self.dy = self.d_old[0] + diff_pos.x(), self.d_old[1] + diff_pos.y()
		self.update()

	def mouseReleaseEvent(self, event):
		self.mouseMoveEvent(event)
