symbol P1 = 0x81
symbol P2 = 0x82
symbol P3 = 0x83
symbol S1 = 0x91
symbol S2 = 0x92
symbol S3 = 0x93

symbol PH1 = 0xA1
symbol PH2 = 0xA2
symbol PH3 = 0xA3
symbol PL1 = 0xB1
symbol PL2 = 0xB2
symbol PL3 = 0xB3

symbol SERIN_PIN  = B.6
symbol SEROUT_PIN = C.0
symbol SER_MODE = T2400

symbol DIR_PIN = pinC.6
symbol PWM_1 = C.5
symbol PWM_2 = C.2
symbol PWM_3 = C.3

symbol AMP_PIN = B.0
symbol TERM1_PIN = B.1
symbol TERM2_PIN = B.2
symbol TERM3_PIN = B.3
symbol CURRENT_PIN = B.4

symbol PWM_PERIOD = 128;

get_val:
	serin [200, no_cmd],SERIN_PIN,SER_MODE,b2
	return

main:
	if DIR_PIN = 0 then
		fvrsetup FVR4096
		adcconfig %011
		serout SEROUT_PIN,SER_MODE,("TEMP")
		readadc TERM1_PIN, b1
	sertxd ("TEMP1 ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
		readadc TERM2_PIN, b1
	sertxd ("TEMP2 ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
		readadc TERM3_PIN, b1
	sertxd ("TEMP3 ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
		serout SEROUT_PIN,SER_MODE,("CURR")
		readadc CURRENT_PIN, b1
	sertxd ("CURRENT ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
	else
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
		case S1
	sertxd ("CMD S1 ", #b1,CR,LF)
			pwmout PWM_1,off
		case S2
	sertxd ("CMD S2 ", #b1,CR,LF)
			pwmout PWM_2,off
		case S3
	sertxd ("CMD S3 ", #b1,CR,LF)
			pwmout PWM_3,off
		case PH1
	sertxd ("CMD PH1 ", #b1,CR,LF)
			high PWM_1
		case PH2
	sertxd ("CMD PH2 ", #b1,CR,LF)
			high PWM_2
		case PH3
	sertxd ("CMD PH3 ", #b1,CR,LF)
			high PWM_3
		case PL1
	sertxd ("CMD PL1 ", #b1,CR,LF)
			low PWM_1
		case PL2
	sertxd ("CMD PL2 ", #b1,CR,LF)
			low PWM_2
		case PL3
	sertxd ("CMD PL3 ", #b1,CR,LF)
			low PWM_3
		else
			goto no_cmd
		endselect
	endif
no_cmd:
	goto main

