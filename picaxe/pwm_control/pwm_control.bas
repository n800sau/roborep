#picaxe 20m2

symbol SERIN_PIN  = B.6
symbol SEROUT_PIN = C.0
symbol SER_MODE = T19200_16 ' for 16Mhz
'symbol SER_MODE = T4800_16 ' for 16Mhz
'symbol SER_MODE = T4800_4 ' for 4Mhz

symbol PWM_1 = B.1
symbol PWM_2 = C.2
symbol PWM_3 = C.3
symbol PWM_4 = C.5
symbol B_5 = C.1
symbol B_6 = C.4
symbol B_7 = C.7
symbol B_8 = B.0
symbol B_9 = B.2
symbol B_10 = B.3
symbol B_11 = B.4
symbol B_12 = B.5
symbol B_13 = B.6
symbol B_14 = B.7


'symbol SERIAL_TIMEOUT = 65535 ' ~ 17 sec (65*4/16, for 16Mhz)
symbol SERIAL_TIMEOUT = 3855 ' ~ 1 sec (3*4/16, for 16Mhz)

' if changing freq then change SER_MODE
' also debug serial output rate (for m16 - 19200, for m32 - 38400 etc)
setfreq m16
'setfreq m4

' PWM period = (period + 1) x 4 x resonator speed (resonator speed for 4MHz = 1/4000000) (0-255)
' PWM duty cycle = (duty) x resonator speed (1-1023)
' Note that the period and duty values are linked by the above equations. If you wish to
' maintain a 50:50 mark-space ratio whilst increasing the period, you must also increase
' the duty cycle value appropriately. A change in resonator will change the formula.
' If you wish to know the frequency, PWM frequency = 1 / (the PWM period)
' freq: 16000000/(4 * (period + 1)) = 4000000/(period+1)
' period = 99 = 40000 Hz = 1/40000 => 0.000025 s
' duty = 0.000025 * 16000000 = 400 (0..400)

main:
'	sertxd ("Main", CR,LF)
	' b1 - command type (P(pwm) or S(off) or B(bit))
	' b2 - pwm index or pin index
	' b3 - pin value
	' command (spaces after numbers are required):
	' pwm on: 'XXP<index> PER<val> DUT<val> '
	' pwm off: 'XXS<index> '
	' pin on: 'XXB<index> L1 '
	' pin off: 'XXB<index> L0 '
	' between each symbol 5-50ms is required
	serin [SERIAL_TIMEOUT, timeout],SERIN_PIN,SER_MODE,("XX"), b1, #b2
'sertxd ("Received", b1,#b2,CR,LF)
	select case b1
		case "P"
			' PER <period>
			' DUT <duty>
			serin [SERIAL_TIMEOUT, timeout],SERIN_PIN,SER_MODE,("PER"),#b3
'sertxd ("Period", #b3,CR,LF)
			serin [SERIAL_TIMEOUT, timeout],SERIN_PIN,SER_MODE,("DUT"),#w8
'sertxd ("Duty", #w8,CR,LF)
'sertxd ("CMD P", #b2,":",#b3,":",#w8,CR,LF)
			select case b2
				case 1
					pwmout PWM_1,b3,w8
				case 2
					pwmout PWM_2,b3,w8
				case 3
					pwmout PWM_3,b3,w8
				case 4
					pwmout PWM_4,b3,w8
			endselect
		case "S"
'sertxd ("CMD S", #b2,CR,LF)
			select case b2
				case 1
					pwmout PWM_1,off
					LOW PWM_1
				case 2
					pwmout PWM_2,off
					LOW PWM_2
				case 3
					pwmout PWM_3,off
					LOW PWM_3
				case 4
					pwmout PWM_4,off
					LOW PWM_4
			endselect
		case "B"
			' L<0-low,1-high>
			serin [SERIAL_TIMEOUT, timeout],SERIN_PIN,SER_MODE,("L"),#b3
'sertxd ("CMD B", #b2,":",#b3,CR,LF)
			select case b2
				case 1
					select case b3
						case 0
							LOW PWM_1
						case 1
							HIGH PWM_1
					endselect
				case 2
					select case b3
						case 0
							LOW PWM_2
						case 1
							HIGH PWM_2
					endselect
				case 3
					select case b3
						case 0
							LOW PWM_3
						case 1
							HIGH PWM_3
					endselect
				case 4
					select case b3
						case 0
							LOW PWM_4
						case 1
							HIGH PWM_4
					endselect
				case 5
					select case b3
						case 0
							LOW B_5
						case 1
							HIGH B_5
					endselect
				case 6
					select case b3
						case 0
							LOW B_6
						case 1
							HIGH B_6
					endselect
				case 7
					select case b3
						case 0
							LOW B_7
						case 1
							HIGH B_7
					endselect
				case 8
					select case b3
						case 0
							LOW B_8
						case 1
							HIGH B_8
					endselect
				case 9
					select case b3
						case 0
							LOW B_9
						case 1
							HIGH B_9
					endselect
				case 10
					select case b3
						case 0
							LOW B_10
						case 1
							HIGH B_10
					endselect
				case 11
					select case b3
						case 0
							LOW B_11
						case 1
							HIGH B_11
					endselect
				case 12
					select case b3
						case 0
							LOW B_12
						case 1
							HIGH B_12
					endselect
				case 13
					select case b3
						case 0
							LOW B_13
						case 1
							HIGH B_13
					endselect
				case 14
					select case b3
						case 0
							LOW B_14
						case 1
							HIGH B_14
					endselect
			endselect
	endselect
	goto main
timeout:
sertxd ("Timeout",CR,LF)
	goto main
