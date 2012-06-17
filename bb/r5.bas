symbol SERVO_V = b.7
symbol SERVO_H = b.5
symbol EYEOUT = c.4
symbol EYETOP = c.2
symbol EYEBOTTOM = c.7
symbol EYELEFT = c.1
symbol EYERIGHT = c.3

symbol LED_1 = c.5
symbol IR_1 = b.4

symbol EYE_L = b0
symbol EYE_R = b1
symbol EYE_T  = b2
symbol EYE_B = b3

symbol F_R = b4

symbol POS_H = b10
symbol POS_V = b11

symbol STEP_H = 2
symbol STEP_V = 2


init:
	setfreq m16
	let POS_H = 150
	let POS_V = 150
	servo SERVO_V, POS_V
	servo SERVO_H, POS_H

main:
	high EYEOUT
	readadc EYELEFT, EYE_L
	let EYE_L = EYE_L - 6
	readadc EYERIGHT, EYE_R
	readadc EYETOP, EYE_T
	let EYE_T = EYE_T - 2
	readadc EYEBOTTOM, EYE_B
	low EYEOUT
	if EYE_L > EYE_R and EYE_L > EYE_T and EYE_L > EYE_B then
		let POS_H = POS_H - STEP_H
	elseif  EYE_R > EYE_T and EYE_R > EYE_B then
		let POS_H = POS_H + STEP_H
	elseif  EYE_T > EYE_B then
		let POS_V = POS_V + STEP_V
	elseif EYE_T < EYE_B then
		let POS_V = POS_V - STEP_V
	endif
	if POS_H < 75 then
		let POS_H = 75
	elseif POS_H > 240 then
		let POS_H = 240
	endif
	servo SERVO_H, POS_H
	if POS_V < 90 then
		let POS_V = 90
	elseif POS_V > 200 then
		let POS_V = 200
	endif
	servo SERVO_V,POS_V
	high LED_1
	readadc IR_1, F_R
	debug
	low LED_1
	goto main
	
