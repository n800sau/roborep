from machine import Pin, Timer
import utime, time

machine.freq(240000000)
print('FREQ', machine.freq())

dir_pin = Pin(16, Pin.OUT)
step_pin = Pin(17, Pin.OUT)
enable_pin = Pin(18, Pin.OUT)
steps_per_revolution = 200

# Initialize timer
tim = Timer()

def step(t):
	global step_pin
	step_pin.value(not step_pin.value())

def rotate_motor(delay):
	# Set up timer for stepping
	tim.init(freq=1000000//delay, mode=Timer.PERIODIC, callback=step)

def loop():
	while True:
		print('Set direction clockwise')
		# Set motor direction clockwise
		dir_pin.value(1)
		enable_pin.value(False)

		print('Spin slowly')
		# Spin motor slowly
		rotate_motor(2)
		utime.sleep_ms(steps_per_revolution*16)
		tim.deinit()  # stop the timer
		utime.sleep(1)

		print('Set direction counterclockwise')
		# Set motor direction counterclockwise
		dir_pin.value(0)

		print('Spin quickly')
		# Spin motor quickly
		rotate_motor(1)
		utime.sleep_ms(steps_per_revolution*16)
		tim.deinit()  # stop the timer
		utime.sleep(1)
		enable_pin.value(True)
#		time.sleep(5)

if __name__ == '__main__':
	loop()
