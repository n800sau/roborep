#picaxe 20m2

' set pwm <duty>
symbol P1 = 0x81
symbol P2 = 0x82
symbol P3 = 0x83
symbol P4 = 0x84
' stop pwm
symbol S1 = 0x91
symbol S2 = 0x92
symbol S3 = 0x93
symbol S4 = 0x94

symbol SERIN_PIN  = B.6
symbol SEROUT_PIN = C.0
symbol SER_MODE = T2400

symbol PWM_1 = C.5
symbol PWM_2 = C.2
symbol PWM_3 = C.3
symbol PWM_4 = B.1

symbol PWM_PERIOD = 128

get_val:
	serin [200, no_cmd],SERIN_PIN,SER_MODE,b2
	return

main:
	serin [100, no_cmd],SERIN_PIN,SER_MODE,("CMD"), b1
	select b1
	case P1
		gosub get_val
		let w8 = b2 * 4
sertxd ("CMD P1 ", #b1,":",#w8,CR,LF)
		pwmout PWM_1,PWM_PERIOD,w8
	case P2
		gosub get_val
		let w8 = b2 * 4
sertxd ("CMD P2 ", #b1,":",#w8,CR,LF)
		pwmout PWM_2,PWM_PERIOD,w8
	case P3
		gosub get_val
		let w8 = b2 * 4
sertxd ("CMD P3 ", #b1,":",#w8,CR,LF)
		pwmout PWM_3,PWM_PERIOD,w8
	case P4
		gosub get_val
		let w8 = b2 * 4
sertxd ("CMD P4 ", #b1,":",#w8,CR,LF)
		pwmout PWM_4,PWM_PERIOD,w8
	case S1
sertxd ("CMD S1 ", #b1,CR,LF)
		pwmout PWM_1,off
	case S2
sertxd ("CMD S2 ", #b1,CR,LF)
		pwmout PWM_2,off
	case S3
sertxd ("CMD S3 ", #b1,CR,LF)
		pwmout PWM_3,off
	case S4
sertxd ("CMD S4 ", #b1,CR,LF)
		pwmout PWM_4,off
	else
		goto no_cmd
	endselect
no_cmd:
	goto main

