import math, time, array
from PyQt4.QtGui import QWidget, QPainter, QPen, QPalette, QColor, QBrush, QGraphicsView, QGraphicsItem, QGraphicsScene
from PyQt4.QtCore import pyqtSignature, Qt, QLine, QPoint, QPointF, QRectF, QRect, QSize, QTimer, pyqtSignal

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
		self.minscale = 10. / min(xs, ys)
		self.maxscale = 50.
		self.matrix = array.array('h', [-1] * xs * ys * zs)
		self.objects = []
		self.scale = 1
		self.robo = Robo()
		self.setRobo(robox, roboy, roboz)
		self.roboSetSpeed(0, 0)

	def rect(self):
		return QRect(0, 0, self.xsize * self.scale, self.ysize * self.scale)

	def setRobo(self, x, y, z):
		self.roboloc = (x, y, z)

	def roboSetSpeed(self, left, right):
		self.roboSpeed = [left, right]

	def roboChangeSpeed(self, dleft, dright):
		self.roboSetSpeed(self.roboSpeed[0] + dleft, self.roboSpeed[1] + dright)

	def posidx(self, x, y, z):
		return x + y * self.xsize + z * self.xsize * self.ysize

	def setScale(self, scale):
		self.scale = scale
		if self.scale < self.minscale:
			self.scale = self.minscale
		elif self.scale > self.maxscale:
			self.scale = self.maxscale

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

	def paint(self, painter, slice_z, ruler_x, ruler_y, viewport):
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
						o.paint(painter, QRectF(x * self.scale, y * self.scale, self.scale, self.scale))
		for x in range(self.xsize):
			xpos = x * self.scale
			# draw ruler
			if (x - ruler_step // 2) % ruler_step == 0 and viewport.left() <= xpos and xpos <= viewport.right():
				painter.drawLine(QPointF(xpos, ruler_y - 3), QPointF(xpos, ruler_y))
				painter.drawText(QPointF(xpos, ruler_y - 20), str(x))
		for y in range(self.ysize):
			ypos = y * self.scale
			# draw ruler
			if (y - ruler_step // 2) % ruler_step == 0  and viewport.top() <= ypos and ypos <= viewport.bottom():
				painter.drawLine(QPointF(ruler_x, ypos), QPointF(ruler_x + 3, ypos))
				painter.drawText(QPointF(ruler_x + 3, ypos), str(y))
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
		self.resetSpeed()
		self.speed = [0, 0]
		self.mPrevPoint = None
		self.animTime = time.time()
		self.animTimer = QTimer()
		self.animTimer.timeout.connect(self.onAnimTimer)
		self.animTimer.start(20)

	def resetSpeed(self):
		self.speed = [0, 0]

	def paintEvent(self, event):
		painter = QPainter(self)
		painter.save()
		try:
			painter.setRenderHint(QPainter.Antialiasing)
			r = self.rect().adjusted(10, 10, -10, -10)
			painter.setClipRect(r)
			wr = self.world.rect()
			xoff = r.center().x() - wr.center().x() + self.dx
			yoff = r.center().y() - wr.center().y() + self.dy
			painter.translate(xoff, yoff)
			self.world.paint(painter, self.slice_z, r.left() - xoff, r.bottom() - yoff, r.adjusted(-xoff, -yoff, -xoff, -yoff))
		finally:
			painter.restore();

	def scaleWorld(self, scale):
		if scale > 0:
			self.world.setScale(scale)
			self.update()

	def moveWorld(self, dx, dy, dz):
		self.dx += dx
		self.dy += dy
		self.slice_z += dz
		self.update()

	def wheelEvent(self, event):
		self.resetSpeed()
		numDegrees =  event.delta() / 16.
		d_scale = (1 + numDegrees/100.)  if numDegrees > 0 else (1 + numDegrees/100.)
		self.scaleWorld(self.world.scale * (d_scale if d_scale> 0 else 1/-d_scale))

	def mousePressEvent(self, event):
		self.resetSpeed()
		self.d_old = (self.dx, self.dy)
		self.mPrevPoint = self.m_pos = event.pos()
		self.prevTime = time.time()

	def mouseMoveEvent(self, event):
		diff_pos = event.pos() - self.m_pos
		self.dx, self.dy = self.d_old[0] + diff_pos.x(), self.d_old[1] + diff_pos.y()
		self.findSpeed(event.pos())
		self.update()

	def mouseReleaseEvent(self, event):
		self.mouseMoveEvent(event)
		self.mPrevPoint = None

	def findSpeed(self, pos):
		duration = time.time() - self.prevTime
		diff = (pos.x() - self.mPrevPoint.x(), pos.y() - self.mPrevPoint.y())
		if duration > 0.01:
			maxspeed = self.world.scale*20
			for i in range(2):
				if diff[i] != 0:
					self.speed[i] = diff[i] / self.world.scale / duration
					if self.speed[i] > maxspeed:
						self.speed[i] = maxspeed
		self.prevTime = time.time()
		self.mPrevPoint = pos

	@pyqtSignature("")
	def onAnimTimer(self):
		t = time.time()
		n = [0, 0]
		if self.mPrevPoint is None and sum(self.speed) != 0:
			tdif = (t - self.animTime)
			for i in range(2):
				if abs(self.speed[i]) > 1:
					n[i] = self.speed[i] * tdif
					#slow down the speed
					next_speed = self.speed[i] - (self.speed[i] * 0.5) * tdif
					self.speed[i] = 0 if next_speed * self.speed[i] < 0 else next_speed
				else:
					self.speed[i] = 0
			self.moveWorld(n[0], n[1], 0)
		self.animTime = t

	def roboLeft(self, val):
		self.world.roboChangeSpeed(val, 0)

	def roboRight(self, val):
		self.world.roboChangeSpeed(0, val)

	def roboF(self, val):
		self.world.roboChangeSpeed(val, val)

	def roboStop(self):
		self.world.roboSetSpeed(0, 0)
