import time, copy, os, sys, json, pickle, cv2, random, redis, subprocess, atexit
from fchassis_ng import fchassis_ng
from utils import angle_diff, html_path, html_data_path
from pids import Pid
from marker import collect_markers
from camera import ShapeSearch, ColorFix
from sensor_const import *


class ExDataTooOld(Exception):
	pass

class ExNoData(Exception):
	pass

class frobo_common(fchassis_ng):


	STEP_TIME = 0.01
	WHOLE_TURN_TIME = 0.5

	MIN_DISTANCE = 0.2

	def __init__(self, *args, **kwds):
		super(frobo_common, self).__init__(*args, **kwds)
		self.r = redis.Redis(db=2)
		self.sensor_process = self.run_sensors_process()
		atexit.register(self.sensor_process.kill)
		self.dbprint('sensor process: %s' % self.sensor_process.pid, force=True)
		for i in range(10):
			try:
				self.heading()
			except (ExDataTooOld, ExNoData):
				self.dbprint('Waiting for sensor2redis.py process, %d sec(s)' % (i+1), force=True)
				time.sleep(1)
				continue
			break
		self.dbprint('sensor2redis.py is producing data')
		self.hit_warn = None
		self.dots = []
		# (int angle): {'lpwr':, 'rpwr':, 'dt':}
		self.memory_fname = os.path.join(os.path.dirname(sys.argv[0]), 'memory.dat')
		self.recover_memory()

	def run_sensors_process(self):
		return subprocess.Popen(['python', os.path.join(os.path.dirname(__file__), 'sensor2redis.py')])

	def heading(self):
		data = self.r.lrange(COMPASS_REDIS_QUEUE, -1, -1)
		rs = json.loads(data[0]) if data else None
		if rs is None:
			raise ExNoData('No data')
		t = time.time()
		if rs['time'] + 2 < t:
			raise ExDataTooOld('Heading data too old (%g sec old)' % (t - rs['time']))
		return rs['heading']

	def accel(self):
		data = self.r.lrange(ACCEL_REDIS_QUEUE, -1, -1)
		rs = json.loads(data[0]) if data else None
		if rs is None:
			raise ExNoData('No data')
		t = time.time()
		if rs['time'] + 2 < t:
			raise ExDataTooOld('Accelerometer data too old (%g sec old)' % (t - rs['time']))
		return rs['axes']

	def degsec(self):
		data = self.r.lrange(GYRO_REDIS_QUEUE, -1, -1)
		rs = json.loads(data[0]) if data else None
		if rs is None:
			raise ExNoData('No data')
		t = time.time()
		if rs['time'] + 2 < t:
			raise ExDataTooOld('Gyro data too old (%g sec old)' % (t - rs['time']))
		return rs['degsec']

	def add_memory(self, angle, lpwr, rpwr, dt):
		self.memory[int(angle)] = {'angle': angle, 'lpwr': lpwr, 'rpwr': rpwr, 'dt': dt}
		self.save_memory()

	def recover_memory(self):
		try:
			self.memory = dict(pickle.load(file(self.memory_fname, 'r')) if os.path.exists(self.memory_fname) else {})
		except:
			self.memory = {}

	def save_memory(self):
		pickle.dump(self.memory, file(self.memory_fname, 'w'))

	def find_memory_closest(self, angle):
		rs = None
		prev_k = None
		for k in self.memory.keys():
			if k > angle:
				diff = abs(k - angle)
				if not prev_k is None:
					if abs(prev_k - angle) < diff:
						k = prev_k
				rs = self.memory[k]
				break
			prev_k = k
		return rs

	def heading_diff(self, azim, err=10):
		heading = self.heading()
		diff = angle_diff(azim, heading)
		return diff if abs(diff) > err else 0

	def update_state(self):
		rs = super(frobo_common, self).update_state()
		if rs:
			self.state['heading'] = self.heading()
			self.state['acc'] = self.accel()
			dot = copy.deepcopy(self.state)
			self.dots.append(dot)
		return rs

	def wait_until_stop(self):
		self.cmd_mstop()
		while not self.is_really_stopped(secs=1):
			pass
		self.dbprint("stopped")

	def is_really_stopped(self, secs=0.5):
		h = self.heading()
		self.update_state()
		lcount = self.state['lcount']
		rcount = self.state['rcount']
		time.sleep(secs)
		self.update_state()
		return lcount == self.state['lcount'] and \
			rcount == self.state['rcount'] and \
			not self.heading_diff(h, err=2)


	def move_straight(self, fwd=True, max_steps=5, max_secs=1, heading=None, power=50, **kwds):
		self.cmd_reset_counters()
		counted_offset = 0
		pid = Pid(2., 0, 0)
		pid.range(-power, power)
		if heading is None:
			heading = self.heading()
		elif abs(angle_diff(heading, self.heading())) > 5:
			self.turn_in_ticks(heading)
		pid.set(heading)
		offset = 0
		try:
			t = time.time()
			while (t + max_secs) > time.time():
				self.update_state()
				hdiff = angle_diff(self.heading(), heading)
				if abs(hdiff) > 20:
					self.dbprint("STOP and CORRECT")
					tt = time.time()
					counted = self.steps_counted()
					self.turn_in_ticks(heading, err=5)
					counted_offset = counted - self.steps_counted()
					self.dbprint("COUNTER OFFSET=%d" % counted_offset)
					hdiff = angle_diff(self.heading(), heading)
					max_secs += time.time() - tt
				if hdiff > 1:
					offset = 20 if fwd else -20
				elif hdiff < -1:
					offset = -20 if fwd else 20
				else:
					offset = 0
				lpwr = power - offset
				rpwr = power + offset
				self.dbprint('^%d hdiff:%g off:%d pw:%d<>%d cnt:%d<>%d' % (int(self.heading()), hdiff, offset, lpwr, rpwr, self.state['lcount'], self.state['rcount']))
				if fwd and self.dist_fwd() >= 0 and self.dist_fwd() < self.MIN_DISTANCE:
					self.hit_warn = self.dist_fwd()
					self.dbprint('STOP distance=%s' % self.dist_fwd())
					break
				steps = self.steps_counted() + counted_offset
				if steps > max_steps:
					self.dbprint('Max steps reached (%d > %d)' % (steps, max_steps))
					hdiff = angle_diff(self.heading(), heading)
					if abs(hdiff) > 3:
						self.dbprint("FINAL CORRECTION")
						self.turn_in_ticks(heading, err=5)
					break
				self.cmd_mboth(lpwr, fwd, rpwr, fwd)
				time.sleep(self.STEP_TIME)
				self.update_state()
				pid.step(input=self.heading())
				offset = pid.get()
		finally:
			self.cmd_mstop()

	# return turn degrees
	def tick_turn_old(self, clockwise, min_angle=None, pwr=50):
		rs = 0
		init_t = time.time()
		init_h = self.heading()
		self.dbprint('start h: %d, pwr: %s' % (init_h, pwr))
		try:
			self.cmd_mboth(pwr, clockwise, pwr, not clockwise)
			for i in range(int(self.WHOLE_TURN_TIME/self.STEP_TIME)):
				h = self.heading()
				x,y,z = self.degsec()
				if abs(z) > 5:
					pwr *= 0.1
					#self.dbprint('moving...%g PWR:%g' % (z, pwr))
					self.cmd_mboth(pwr, clockwise, pwr, not clockwise)
				if min_angle is None:
					x,y,z = self.degsec()
					self.dbprint('g:%d %d %d, h:%d' % (x, y, z, h))
					if abs(z) > 15:
						self.dbprint('it moves')
						break
				else:
					hdiff = angle_diff(h, init_h)
					#self.dbprint('h=%g, hdiff=%g' % (h, hdiff))
					if abs(hdiff) > min_angle:
						self.dbprint('%g degrees trigger stop' % hdiff)
						break
			#	self.update_state()
				time.sleep(self.STEP_TIME)
		finally:
			self.cmd_mstop()
			self.wait_until_stop()
			h = self.heading()
			self.add_memory(abs(init_h - h), pwr, pwr, time.time() - init_t)
			rs = init_h - h
			self.dbprint('last h: %d, change: %g' % (h, rs))
		return rs

	# return turn degrees
	def tick_turn(self, clockwise, min_angle=None, pwr=50, mleft=True, mright=True):
		rs = 0
		max_pwr = pwr
		in_t = time.time()
		in_h = self.heading()
		if not min_angle is None:
			azim = (in_h + min_angle) if clockwise else (in_h - min_angle)
		in_x,in_y,in_z = self.degsec()
		self.dbprint('start h: %.2f, z: %.2f, pwr: %s' % (in_h, in_z, pwr))
		moved = False
		try:
			while not moved:
				if mleft and mright:
					self.cmd_mboth(pwr, clockwise, pwr, not clockwise)
				elif mleft:
					self.cmd_mleft(pwr, clockwise)
				elif mright:
					self.cmd_mright(pwr, not clockwise)
				for i in range(int(self.WHOLE_TURN_TIME/self.STEP_TIME)):
	#				time.sleep(self.STEP_TIME)
					dt = time.time() - in_t
					h = self.heading()
					x,y,z = self.degsec()
	#				self.dbprint('dh: %.2f, dz: %.2f, dt:%.2f' % (h - in_h, z - in_z, dt))
					if min_angle is None:
						self.dbprint('g:%d %d %d, h:%.2f' % (x, y, z, h))
						if abs(z) >= 2:
							self.dbprint('it moves')
							moved = True
							break
					else:
						# remaining angle
						adiff = angle_diff(azim, h)
						# passed angle
						pdiff = angle_diff(h, in_h)
						# f(dt, pwr, z) = 
						# remain distance depends on z, mass and friction
						# f((z, dt) - friction?
						# (z / dt) - acceleration = K * (pwr - friction)
						# time to finish
						# curr angle velocity (with offset)
						if z - in_z != 0:
							rt = abs(adiff / (z - in_z))
							if rt < dt:
								moved = True
								# stop
								self.dbprint('Moved FAST enough %.2f < %.2f' % (rt, dt))
								self.cmd_mstop()
								self.wait_until_stop()
								h = self.heading()
								adiff = min_angle - abs(h - in_h)
								break
				if not moved:
					pwr += 5
					if pwr > 100:
						self.dbprint('NOT MOVED and STUCK')
						break
					self.dbprint('NOT MOVED power now gets: %s' % (pwr,))
		finally:
			self.cmd_mstop()
# update_state takes too long
			self.update_state()
			self.wait_until_stop()
			self.update_state()
			h = self.heading()
			self.add_memory(abs(in_h - h), pwr, pwr, time.time() - in_t)
			rs = in_h - h
			self.dbprint('last h: %.2f, CHANGE: %g' % (h, rs))
		return {'change': rs, 'pwr': pwr}

	def tick_left(self, min_angle=None, pwr=40):
		return self.tick_turn(False, min_angle=min_angle, pwr=pwr)

	def tick_right(self, min_angle=None, pwr=40):
		return self.tick_turn(True, min_angle=min_angle, pwr=pwr)

	def simple_turn(self, azim, err=3, pwr=90, cb_func=None, precise_stop=True):
		diff = self.heading_diff(azim, err=err)
		if diff != 0:
			clockwise = diff > 0
			self.dbprint('heading:%d, diff:%d, simple turn %s' % (self.heading(), diff, ('clockwise' if clockwise else 'counter-clockwise')))
			self.cmd_mboth(pwr, clockwise, pwr, not clockwise)
			i = 0
			while diff != 0 and (diff > 0) == clockwise and i < int(self.WHOLE_TURN_TIME/self.STEP_TIME):
				time.sleep(self.STEP_TIME)
				diff = self.heading_diff(azim, err=err)
				if not cb_func is None:
					cb_func(self)
				i += 1
			# finished
			self.cmd_mstop()
			self.wait_until_stop()
			if precise_stop:
				diff = self.heading_diff(azim, err=err)
				if diff != 0:
					# it is gone too far
					# apply the old way
					self.dbprint('Run turn in ticks as a fallback')
					self.turn_in_ticks(azim, err=err)

	def turn_using_memory(self, azim, err=3, stop_if=None, move_cb=None):
		diff = self.heading_diff(azim, err=err)
		if diff:
			data = self.find_memory_closest(azim)
			if data is None:
				self.turn_in_ticks(azim, err=err, stop_if=stop_if, move_cb=move_cb)
			else:
				self.dbprint("Using memory %s to turn %s degrees" % (data, diff))
				lpwr = data['lpwr']
				rpwr = data['rpwr']
				dt = data['dt'] * (abs(diff) / max(1, data['angle']))
				self.dbprint("Wait for %g" % dt)
				clockwise = diff > 0
				self.cmd_mboth(lpwr, clockwise, rpwr, not clockwise)
				time.sleep(dt)
				self.cmd_mstop()
				self.wait_until_stop()
				diff = self.heading_diff(azim, err=err)
				if diff:
					self.dbprint("Now using turn_in_ticks for the rest %s" % diff)
					self.turn_in_ticks(azim, err=err, stop_if=stop_if, move_cb=move_cb)

	def turn_in_ticks(self, azim, err=3, stop_if=None, move_cb=None):
		min_pwr = 30
		max_pwr = 100
		slow_coef = 1
		change_list = []
		self.dbprint('start turn h %d' % int(self.heading()))
		try:
			last_counts = [self.state['lcount'], self.state['rcount'], 0]
			last_diff = self.heading_diff(azim, err=err)
			for i in range(360):
				diff = self.heading_diff(azim, err=err)
				adiff = abs(diff)
				if adiff < err:
					break
				if (bool(diff>0) ^ bool(last_diff>0)):
					# missed the point
					# slow down
					slow_coef -= 0.1
					self.dbprint('SIGN CHANGE, coef=%g' % slow_coef)
				pwr = (min_pwr + (max_pwr - min_pwr) * (adiff / 180.)) * max(0.1, slow_coef)
				self.dbprint('######################### POWER %g , diff=%g' % (pwr, diff))
				if adiff < 30:
					if diff > 0:
						move_data = self.tick_right(pwr = pwr)
					else:
						move_data = self.tick_left(pwr = pwr)
				else:
					if diff > 0:
						move_data = self.tick_right(min_angle=adiff, pwr = pwr)
					else:
						move_data = self.tick_left(min_angle=adiff, pwr = pwr)
				change = move_data['change']
				if (abs(change) < 1 or pwr < move_data['pwr']) and slow_coef < 1:
					slow_coef += 0.1
#					min_pwr += 5
#					if min_pwr > max_pwr - 10:
#						min_pwr = max_pwr - 10
				change_list.append(change)
				change_list = change_list[-10:]
				avg_change = sum(change_list)/len(change_list)
				max_change = max(change_list)
				min_change = min(change_list)
				self.dbprint("avg:%g, min:%g, max:%g" % (avg_change, min_change, max_change))
#				if (max_change > 0) != (min_change > 0):
					# close to zero, slow down
#					slow_coef *= 0.9
#					if slow_coef < 0.1:
#						slow_coef = 0.1
#					change_list = []
				self.db_state()
				if last_counts[0] == self.state['lcount'] and last_counts[1] == self.state['rcount']:
					last_counts[2] += 1
					if last_counts[2] > int(1/self.STEP_TIME):
						self.dbprint('Stopped moving')
						break
				else:
					if move_cb:
						move_cb(self)
					last_counts = [self.state['lcount'], self.state['rcount'], 0]
				if stop_if:
					if stop_if(self):
						break
				last_diff = diff
		finally:
			self.cmd_mstop()
			self.dbprint('end turn h %d' % int(self.heading()))

	def search_around(self, stop_if_cb, clockwise=True, pwr=30, step_angle=10, max_pwr=100):
		min_pwr = pwr
		ih = self.heading()
		last_diff = 0
		growing = True
		for step in range(10000):
			change = self.tick_turn(clockwise, pwr=pwr, min_angle=step_angle)['change']
			if abs(change) < 1:
				pwr += 5
				if pwr > max_pwr - 10:
					pwr = max_pwr - 10
			else:
				pwr = min_pwr
			self.wait_until_stop()
			h = self.heading()
			adiff = abs(angle_diff(ih, h))
			if growing:
				if adiff < last_diff - 5:
					growing = False
			else:
				if adiff > last_diff + 5:
					break
			last_diff = adiff
			if stop_if_cb(self):
				break

	def stop_if_cb(self, stop_dist):
		n = 10
		self.update_state()
		while self.dist_fwd() < 0 and n > 0:
			time.sleep(0.01)
			self.update_state()
			n -= 1
		return self.dist_fwd() > stop_dist if self.dist_fwd() >= 0 else False

	def find_distance(self, min_dist=100, clockwise=True):
		self.stop_if_cb(1)
		# 10 attempt to turn
		i = 10
		while self.dist_fwd() < min_dist and i>0:
			self.turn_in_ticks(self.heading() + (45 if clockwise else -45), stop_if=lambda c: c.stop_if_cb(min_dist))
			i -= 1

	def find_left_minimum(self, fwd=True):
		return self.find_pwr_minimum('left', fwd)

	def find_right_minimum(self, fwd=True):
		return self.find_pwr_minimum('right', fwd)

	def find_pwr_minimum(self, motor, fwd):
		rs = None
		func = self.cmd_mright if motor[0] == 'r' else self.cmd_mleft
		try:
			for pwr in range(100):
				func(pwr, fwd)
				for i in range(int(0.2 / self.STEP_TIME)):
					time.sleep(self.STEP_TIME)
					x,y,z = self.degsec()
					if abs(z) > 10:
						self.dbprint('it moves')
						rs = pwr
						break
				if not rs is None:
					break
		finally:
			self.cmd_mstop()
		return rs


	def search_marker(self, r, clockwise=True, marker_id=None):

		def collect_markers_cb(r, marker_id):
			class do_it:

				def __init__(self):
					self.i = 0;
					self.fnames = []
					self.target_loc = None

				def __call__(self, c):
					rs = False
					fname = html_data_path('pic%d.jpg' % self.i)
					markers = collect_markers(r, fpath=fname)
					if markers:
						c.dbprint('FOUND %d markers (%d):' % (len(markers), c.heading()))
						self.fnames.append(fname)
						for m in markers:
							if marker_id is None:
								c.dbprint('\t%s' % (m['id'], ))
							else:
								c.dbprint('\t%s vs %s' % (m['id'], marker_id))
							#	os.system('espeak "%s"' % ' '.join([c for c in str(m['id'])]))
							if marker_id and m['id'] == marker_id:
								self.target_loc = m
								rs = True
								break
						self.i += 1
					return rs

			return do_it()

		cb = collect_markers_cb(r, marker_id)
		self.search_around(cb, clockwise=clockwise)
		json.dump({'files': cb.fnames, 'title': 'markers', 'time_mark': int(time.time())},
			file(html_path('frames.json'), 'w'), indent=2)
		return cb.target_loc

	def search_shapes(self, camera, clockwise=True):

		ss = ShapeSearch(camera)

		def search_cb():
			class do_it:

				def __init__(self):
					self.i = 0;
					self.fnames = []

				def __call__(self, c):
					data = ss.find_shapes(threshold=0, resolution=(160, 120))
					if data:
						if os.environ.get('RASPICAM_ROTATE', ''):
							angle = int(os.environ['RASPICAM_ROTATE'])
							rows,cols,depth = data['frame'].shape
							M = cv2.getRotationMatrix2D((cols/2,rows/2), 180, 1)
							data['frame'] = cv2.warpAffine(data['frame'], M, (cols,rows))
						fname = data_path('shapes_%04d.jpg' % self.i)
						self.fnames.append(fname)
						cv2.imwrite(html_path(fname), data['frame'])
						fname = html_data_path('thresh_%04d.jpg' % self.i)
						self.fnames.append(fname)
						cv2.imwrite(html_path(fname), data['thresh'])
						c.dbprint('Written set %d' % self.i)
						self.i += 1

			return do_it()

		cb = search_cb()
		self.search_around(cb, clockwise=clockwise)
		json.dump({'files': cb.fnames, 'title': 'shapes', 'time_mark': int(time.time())},
			file(html_path('frames.json'), 'w'), indent=2)

	def search_contours(self, camera, clockwise=True):

		ss = ColorFix(camera)

		def search_cb():
			class do_it:

				def __init__(self):
					self.i = 0;

				def __call__(self, c):
					data = ss.contours()
					if data:
						cv2.imwrite(html_data_path('frame_%03d.jpg' % self.i), data['frame'])
						cv2.imwrite(html_data_path('iframe_%03d.jpg' % self.i), data['iframe'])
						cv2.imwrite(html_data_path('oframe_%03d.jpg' % self.i), data['oframe'])
						c.dbprint('Written set %d' % self.i)
						self.i += 1

			return do_it()

		cb = search_cb()
		self.search_around(cb, clockwise=clockwise)
		json.dump({'imgcount': cb.i}, file(html_path('frames.json'), 'w'), indent=2)

	def collect_turn_data(self, cnt=100, clockwise=None, dT=None, pwr=None):
		data = []
		random.seed()
		for i in range(cnt):
			_clockwise = random.choice((True, False)) if clockwise is None else clockwise
			_pwr = random.randint(1, 100) if pwr is None else pwr
			_dT = random.uniform(0.1, 3) if dT is None else dT
			h = self.heading()
			self.dbprint('Test pwr:%d, dT:%.1f, cw: %s' % (_pwr, _dT, _clockwise))
			self.cmd_mboth(_pwr, _clockwise, _pwr, not _clockwise)
			time.sleep(_dT)
			self.cmd_mstop()
			self.wait_until_stop()
			h1 = self.heading()
			adiff = abs(angle_diff(h, self.heading()))
			self.dbprint('Result:%.2f' % adiff)
			data.append({'output': {'adiff': adiff}, 'input': {'pwr': _pwr, 'dT': _dT, 'clockwise': _clockwise}})
		return data

	def mask_images(self, camera, hsvLower, hsvUpper, clockwise=True):

		ss = ColorFix(camera)

		def search_cb():
			class do_it:

				def __init__(self):
					self.i = 0;

				def __call__(self, c):
#					data = ss.colorise()
#					data = ss.colorise1()
#					data = ss.locate_object(hsvLower, hsvUpper)
					data = ss.mask_range(hsvLower, hsvUpper)
					if data:
						cv2.imwrite(html_data_path('frame_%03d.jpg' % self.i), data['frame'])
						cv2.imwrite(html_data_path('iframe_%03d.jpg' % self.i), data['iframe'])
						cv2.imwrite(html_data_path('oframe_%03d.jpg' % self.i), data['oframe'])
						c.dbprint('Written set %d' % self.i)
						self.i += 1

			return do_it()

		cb = search_cb()
		self.search_around(cb, clockwise=clockwise)
		json.dump({'imgcount': cb.i}, file(html_path('frames.json'), 'w'), indent=2)

	def dist_fwd(self):
		return self.state['sonar']
#		dists = [dist for dist in (self.state['sonar'], self.state['irdist']) if dist >= 0]
#		return min(dists if dists else -1)
