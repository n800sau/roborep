#picaxe 20m2

symbol SERIN_PIN  = B.6
symbol SEROUT_PIN = C.0
symbol SER_MODE = T4800_16 ' for 16Mhz

symbol PWM_1 = B.1
symbol PWM_2 = C.2
symbol PWM_3 = C.3
symbol PWM_4 = C.5

symbol TIMEOUT_BEFORE_OFF = 65535 ' ~ 17 sec (65*4/16, for 16Mhz)

' if changing freq then change SER_MODE
' also debug serial output rate (for m16 - 19200, for m32 - 38400 etc)
setfreq m16

main:
	' b1 - command type (P(pwm) or S(off) or B(bit))
	' b2 - pwm index or pin index
	serin [TIMEOUT_BEFORE_OFF, turn_off],SERIN_PIN,SER_MODE,("PWM"), b1, #b2
'sertxd ("Received", b1,#b2,CR,LF)
	select case b1
		case "P"
			' PER <period>
			' DUT <duty>
			serin [200, turn_off],SERIN_PIN,SER_MODE,("PER"),#b3
'sertxd ("Period", #b3,CR,LF)
			serin [200, turn_off],SERIN_PIN,SER_MODE,("DUT"),#w8
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
			' ST <0-low,1-high>
sertxd ("CMD B", #b2,CR,LF)
			serin [200, turn_off],SERIN_PIN,SER_MODE,("ST"),#b3
			select case b2
				case 1
					b2 = PWM_1
				case 2
					b2 = PWM_2
				case 3
					b2 = PWM_3
				case 4
					b2 = PWM_4
				default:
					b2 = 0
			endselect
			if b2 <> 0 then
				select case b3
					case 0
						LOW b2
					case 1
						HIGH b2
				endselect
			end if
	endselect
	goto main
turn_off:
	' turn off all pwn of timeout
sertxd ("Timeout, all off",CR,LF)
	pwmout PWM_1,off
	pwmout PWM_2,off
	pwmout PWM_3,off
	pwmout PWM_4,off
	goto main
