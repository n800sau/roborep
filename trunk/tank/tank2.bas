symbol M1_PWM = c.5
symbol M1_DIR = c.4
symbol M2_PWM = c.3
symbol M2_DIR = c.2
symbol MT_PWM = b.1
symbol MT_DIR = b.0
symbol LIGHT_LEFT = b.2
symbol LIGHT_RIGHT = b.3
symbol BLIGHT_LEFT= b.4
symbol BLIGHT_RIGHT=c.1

symbol SERINPIN = b.5
symbol SEROUTPIN = b.7

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
	output	LIGHT_LEFT
	output	LIGHT_RIGHT
	output	BLIGHT_LEFT
	output	BLIGHT_RIGHT
	low	LIGHT_LEFT
	low	LIGHT_RIGHT
	low	BLIGHT_LEFT
	low	BLIGHT_RIGHT

main:
	serin [1000, main], SERINPIN, N19200_32, ("CMD_BOTTOM"), CMD, #NUM
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
		endselect
		serout SEROUTPIN, N19200_32, ("REPLY_BOTTOM", CMD, #RNUM)
	goto main
