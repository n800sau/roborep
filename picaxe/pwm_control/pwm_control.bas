#picaxe 20m2

symbol SERIN_PIN  = B.6
symbol SEROUT_PIN = C.0
symbol SER_MODE = T4800_16 ' for 16Mhz
'symbol SER_MODE = T4800_4 ' for 4Mhz

symbol PWM_1 = B.1
symbol PWM_2 = C.2
symbol PWM_3 = C.3
symbol PWM_4 = C.5
symbol B_5 = C.1
symbol B_6 = C.4
symbol B_7 = C.7
symbol B_8 = B.0
symbol B_9 = B.1
symbol B_10 = B.2
symbol B_11 = B.3
symbol B_12 = B.4
symbol B_13 = B.5
symbol B_14 = B.7


symbol TIMEOUT_BEFORE_OFF = 65535 ' ~ 17 sec (65*4/16, for 16Mhz)

' if changing freq then change SER_MODE
' also debug serial output rate (for m16 - 19200, for m32 - 38400 etc)
setfreq m16
'setfreq m4

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
	' between each symbol 50ms is required
	serin [TIMEOUT_BEFORE_OFF, timeout],SERIN_PIN,SER_MODE,("XX"), b1, #b2
'sertxd ("Received", b1,#b2,CR,LF)
	select case b1
		case "P"
			' PER <period>
			' DUT <duty>
			serin [1000, timeout],SERIN_PIN,SER_MODE,("PER"),#b3
'sertxd ("Period", #b3,CR,LF)
			serin [1000, timeout],SERIN_PIN,SER_MODE,("DUT"),#w8
'sertxd ("Duty", #w8,CR,LF)
sertxd ("CMD P", #b2,":",#b3,":",#w8,CR,LF)
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
sertxd ("CMD S", #b2,CR,LF)
			select case b2
				case 1
					pwmout PWM_1,off
				case 2
					pwmout PWM_2,off
				case 3
					pwmout PWM_3,off
				case 4
					pwmout PWM_4,off
			endselect
		case "B"
			' L<0-low,1-high>
			serin [1000, timeout],SERIN_PIN,SER_MODE,("L"),#b3
sertxd ("CMD B", #b2,":",#b3,CR,LF)
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
