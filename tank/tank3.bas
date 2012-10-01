symbol M1_PWM = c.5
symbol M1_DIR = c.4
symbol M2_PWM = c.3
symbol M2_DIR = c.2
symbol MT_PWM = b.1
symbol MT_DIR = b.0

symbol V_BATT1 = b.2
symbol V_SW1 = b.3
symbol V_BATT2 = b.4
symbol V_SW2 = b.5

'b.6,b.5
symbol CMD = b0
symbol NUM = w1
symbol RNUM = w2

'temporary vars
symbol TW1 = w3
symbol TW1_0 = b6
symbol TW1_1 = b7
symbol TW2 = w4
symbol TW2_0 = b8
symbol TW2_1 = b9
symbol TW3 = w5
symbol TW3_0 = b10
symbol TW3_1 = b11

setfreq m32

init:	
	pwmout M1_PWM,150,0
	pwmout M2_PWM,150,0
	pwmout MT_PWM,150,0
	output M1_DIR
	output M2_DIR
	output MT_DIR
	low		M1_DIR
	low		M2_DIR
	low		MT_DIR
	input	V_BATT1
	input	V_SW1
	input	V_BATT2
	input	V_SW2
	'set FVR as 2.048V
	fvrsetup FVR2048
	'set FVR as ADC Vref+, 0V Vref-
	adcconfig %011
	let TW3 = 1

main:
	serout SEROUTPIN, N19200_32, ("TIMEOUT")
	let TW3 = TW3 + 1
	let b1 = 111
	debug
	serin [60000], SERINPIN, N19200_32, CMD
	let b1 = 222
	let b12=CMD
	debug
	serin [60000], SERINPIN, N19200_32, #NUM
	let b1 = 333
	let b13=NUM
	debug
	let RNUM = 0
	select case CMD
		case "f"
			pwmduty M1_PWM, NUM
			pwmduty M2_PWM, NUM
			if NUM < 0 then
				low M1_DIR
				low M2_DIR
			else
				high M1_DIR
				high M2_DIR
			endif
			let RNUM = 1
		case "l"
			pwmduty M1_PWM, NUM
			if NUM < 0 then
				low M1_DIR
			else
				high M1_DIR
			endif
			let RNUM = 1
		case "r"
			pwmduty M2_PWM, NUM
			if NUM < 0 then
				low M2_DIR
			else
				high M2_DIR
			endif
			let RNUM = 1
		case "t"
			pwmduty MT_PWM, NUM
			if NUM < 0 then
				low MT_DIR
			else
				high MT_DIR
			endif
			let RNUM = 1
		case "s"
			low		M1_DIR
			low		M2_DIR
			low		MT_DIR
			pwmout M1_PWM,150,0
			pwmout M2_PWM,150,0
			pwmout MT_PWM,150,0
			let RNUM = 1
		case "<"
			if NUM <= 0 then
				low LIGHT_LEFT
			else
				high LIGHT_LEFT
			endif
			let RNUM = 1
		case ">"
			if NUM <= 0 then
				low LIGHT_RIGHT
			else
				high LIGHT_RIGHT
			endif
			let RNUM = 1
		case "{"
			if NUM <= 0 then
				low BLIGHT_LEFT
			else
				high BLIGHT_LEFT
			endif
			let RNUM = 1
		case "}"
			if NUM <= 0 then
				low BLIGHT_RIGHT
			else
				high BLIGHT_RIGHT
			endif
			let RNUM = 1
		case "c"
			readadc10 V_BEFORE, TW1
			let TW1 = TW1 * 2048 / 1023
			readadc10 V_AFTER, TW2
			let TW2 = TW2 * 2048 / 1023
			let RNUM = TW1 - TW2
			let RNUM = RNUM * 20
		case "v"
			readadc10 V_BEFORE, TW1
			let RNUM = TW1 * 2048 / 1023
		endselect
		serout SEROUTPIN, N19200_32, ("REPLY_BOTTOM", CMD, #RNUM)
	goto main

