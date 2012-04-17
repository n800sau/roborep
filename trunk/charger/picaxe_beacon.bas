'beacon sends i1, lowes power then i2, then lowers and i3 etc
'when 3pi receives all of them then it is close and in direction
'power is chaged by additional picaxe pin

'The PWM period = (period + 1) x 4 x resonator speed (resonator speed for 4MHz = 1/4000000).
'The PWM duty cycle = (duty) x resonator speed

symbol ir_cmd = b6
symbol PWM_PIN = c.2
symbol SERVO_PIN = c.1
symbol IROUT_PIN = c.4
symbol IRIN_PIN = c.3
symbol DAC_PIN = c.0
symbol timeout=500

main:
	setfreq m8
mainloop:
	pwmout PWM_PIN, 150, 0
    pause 5
    irout IROUT_PIN, 1, 1
    pause timeout

	pwmout PWM_PIN, 150, 100
    pause 5
    irout IROUT_PIN, 1, 2
    pause timeout

	pwmout PWM_PIN, 150, 200
    pause 5
    irout IROUT_PIN, 1, 3
    pause timeout

	pwmout PWM_PIN, 150, 400
    pause 5
    irout IROUT_PIN, 1, 4
    pause timeout

	pwmout PWM_PIN, 150, 600
    pause 5
    irout IROUT_PIN, 1, 5
    pause timeout
    
	irin [100, mainloop], IRIN_PIN, ir_cmd
	if ir_cmd = 1 then lockon
	if ir_cmd = 2 then lockoff
	goto mainloop

lockon:
	servo SERVO_PIN, 500
	goto mainloop

lockoff:
	servo SERVO_PIN, 200
	goto mainloop
