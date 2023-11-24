'needs 2 PWM, 4 ADC, 4 digital output, 2 serial or 2 i2c

symbol RIGHT_PWM = c.0
symbol RIGHT_DIR = c.1
symbol LEFT_PWM = c.0
symbol LEFT_DIR = c.4

symbol LED_RF = c.5
symbol IR_RF = b.2

symbol LED_LF = c.5
symbol IR_LF = b.3

symbol LED_RR = c.5
symbol IR_RR = b.4

symbol LED_LR = c.5
symbol IR_LR = b.5

symbol F_RF = b4
symbol F_LF = b5

symbol F_RR = b6
symbol F_LR = b7

symbol t1 = b1

symbol TOO_CLOSE = 200

init:
	setfreq m16

main:
	high LED_RF
	readadc IR_RF, F_RF
	low LED_RF

	high LED_LF
	readadc IR_LF, F_LF
	low LED_LF

	high LED_RR
	readadc IR_RR, F_RR
	low LED_RR

	high LED_LR
	readadc IR_LR, F_LR
	low LED_LR

	if F_RF > TOO_CLOSE then
		high RIGHT_DIR
		pwmout RIGHT_PWM, 150, 800
	elseif F_RR > TOO_CLOSE
		low RIGHT_DIR
		pwmout RIGHT_PWM, 150, 800
	else
		pwmout RIGHT_PWM, 150, 0
	endif

	if F_LF > TOO_CLOSE then
		high LEFT_DIR
		pwmout LEFT_PWM, 150, 800
	elseif F_LR > TOO_CLOSE
		low LEFT_DIR
		pwmout LEFT_PWM, 150, 800
	else
		pwmout LEFT_PWM, 150, 0
	endif

	goto main

turn:
	random t1
	if t1 > 3000:
		'turn left
	else:
		'turn right
	return
