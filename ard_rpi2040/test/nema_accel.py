from machine import Pin, Timer
import utime

class Stepper:
	def __init__(self, dir_pin, step_pin, en_pin):
		self.dir_pin = Pin(dir_pin, Pin.OUT)
		self.step_pin = Pin(step_pin, Pin.OUT)
		self.en_pin = Pin(en_pin, Pin.OUT)
		self.en_pin.value(0)
		self.position = 0

	def move(self, steps, delay, accel):
		self.dir_pin.value(0 if steps > 0 else 1)
		steps = abs(steps)
		for i in range(steps):
			self.step_pin.value(1)
			utime.sleep_us(delay)
			self.step_pin.value(0)
			utime.sleep_us(delay)
			if i < steps // 2 and delay > 100:
				delay -= accel
			elif i >= steps // 2 and delay < 2000:
				delay += accel
		self.position += steps if steps > 0 else -steps

stepper = Stepper(16, 17, 18)

def loop():
	ms_multiply = 16
	MAX_SPEED = 3000
	MIN_SPEED = 50
	speed = MIN_SPEED
	while True:
		stepper.move(200 * ms_multiply, 1000000//speed//ms_multiply, 0)  # 2 revolutions forward
		utime.sleep(1)
		stepper.move(-200 * ms_multiply, 1000000//speed//ms_multiply, 0)  # 2 revolutions backward
		utime.sleep(1)

if __name__ == '__main__':
	loop()
