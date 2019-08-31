' set pwm <duty>
symbol P1 = 0x81
symbol P2 = 0x82
symbol P3 = 0x83
' stop pwm
symbol S1 = 0x91
symbol S2 = 0x92
symbol S3 = 0x93

' set high
symbol PH1 = 0xA1
symbol PH2 = 0xA2
symbol PH3 = 0xA3
' set low
symbol PL1 = 0xB1
symbol PL2 = 0xB2
symbol PL3 = 0xB3

' read data
symbol GD = 0xC1

symbol SERIN_PIN  = B.6
symbol SEROUT_PIN = C.0
symbol SER_MODE = T2400

symbol PWM_1 = C.5
symbol PWM_2 = C.2
symbol PWM_3 = C.3

symbol TERM1_PIN = B.1
symbol TERM2_PIN = B.2
symbol TERM3_PIN = B.3
symbol CURRENT_PIN = B.4

symbol PWM_PERIOD = 128

symbol R1 = 26800

get_val:
	serin [200, no_cmd],SERIN_PIN,SER_MODE,b2
	return

' b8 - pin
' w5 - accamulator
' b1 - temp
read_adc:
	sertxd("READ START ")
	w5 = 0
	for b9 = 1 to 10
		readadc b8, b1
		sertxd(#b8, ":", #w5, "+", #b1)
		let w5 = w5 + b1
		sertxd("=", #w5, " ")
	next b9
	let b1 = w5 + 5 / 10
	sertxd("STOP ", #w5,CR,LF)
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
	case GD
		fvrsetup FVR4096
		adcconfig %011
		serout SEROUT_PIN,SER_MODE,("T")
'		readadc TERM1_PIN, b1
		b8 = TERM1_PIN
		gosub read_adc
	sertxd ("TEMP1 ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
		b8 = TERM2_PIN
		gosub read_adc
	sertxd ("TEMP2 ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
		b8 = TERM3_PIN
		gosub read_adc
	sertxd ("TEMP3 ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
		serout SEROUT_PIN,SER_MODE,("C")
	sertxd ("CURRENT ", #b1,CR,LF)
		serout SEROUT_PIN,SER_MODE,(b1)
	else
		goto no_cmd
	endselect
no_cmd:
	goto main

