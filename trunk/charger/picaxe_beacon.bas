'beacon sends i1, lowes power then i2, then lowers and i3 etc
'when 3pi receives all of them then it is close and in direction
'power is chaged by additional picaxe pin

'The PWM period = (period + 1) x 4 x resonator speed (resonator speed for 4MHz = 1/4000000).
'The PWM duty cycle = (duty) x resonator speed

symbol ir_cmd = b6
symbol i = b0
symbol pwd = w1

symbol PWM_PIN = c.2
symbol SERVO_PIN = c.1
symbol IROUT_PIN = c.4
symbol IRIN_PIN = c.3
symbol DAC_PIN = c.0
symbol timeout=500

main:
	setfreq m4
mainloop:
	'the bigger i and bigger pwm voltage the lower led power
	for i = 0 to 10
		let pwr = i * 60
		pwmout PWM_PIN, 150, pwr
		ir_cmd = 0
		'timeout for the capacitor
		irin [timeout], IRIN_PIN, ir_cmd
		if ir_cmd = 1 then
			servo SERVO_PIN, 500
		endif
		if ir_cmd = 2 then
			servo SERVO_PIN, 200
		endif
		'repeate ir command a few times
		irout IROUT_PIN, 1, i
		irout IROUT_PIN, 1, i
		irout IROUT_PIN, 1, i
		irout IROUT_PIN, 1, i
    next i
	goto mainloop
