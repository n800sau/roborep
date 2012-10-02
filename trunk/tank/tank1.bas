symbol M1_PWM = c.5
symbol M1_DIR = c.4
symbol M2_PWM = c.3
symbol M2_DIR = c.2
symbol MT_PWM = b.1
symbol MT_DIR = b.0
'i2c : b.5 , b.7
symbol LIGHT_LEFT = b.2
symbol LIGHT_RIGHT = b.3
symbol BLIGHT_LEFT= b.4
symbol BLIGHT_RIGHT=c.1

symbol CMD_LOC = 128
symbol PARM_START= CMD_LOC + 1
symbol PARM_END = CMD_LOC + 10 '138
symbol IDLE_TASK_LOC = PARM_END + 1

symbol CMD = b1
symbol NUM = w3
symbol REVR = b2

'temporary vars
symbol TB1 = b8
symbol TB2 = b9
symbol TB3 = b10
symbol TB4 = b11
symbol TW1 = w6
symbol TW1_0 = b12
symbol TW1_1 = b13
symbol TW2 = w7
symbol TW2_0 = b14
symbol TW2_1 = b15
symbol TW3 = w8
symbol TW3_0 = b16
symbol TW3_1 = b17

symbol DIRNUM = w10

setfreq m32

init:	
		hsersetup B19200_32 ,%00			; baud 19200 at 32MHz
		pwmout M1_PWM,150,0
		pwmout M2_PWM,150,0
		pwmout MT_PWM,150,0
'		output M1_PWM
'		output M2_PWM
'		low		M1_PWM
'		low 	M2_PWM
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
'		gosub  test_lights
		
		'high 400 -  0
		'low 200-1023
'		pwmduty       M1_PWM, 1023
'		pwmduty M2_PWM,200


main:
	gosub readcmd
	debug TB1
	if TB1 > CMD_LOC then
		gosub parsecmd
		gosub executecmd
	endif
	goto main
		
parsecmd:
	peek CMD_LOC, CMD
	let NUM = 0
	let REVR = 0
	for TB1 = PARM_START to PARM_END
		peek TB1, TB2
		if TB1 = PARM_START and TB2 = "-" then
			let REVR = 1
		else
			if TB2 <"0" or TB2 > "9" then
				exit
			endif
			let NUM = NUM * 10 + TB2 - "0"
		endif
	next
	if NUM > 1000 then
		NUM = 1000
	endif
	if REVR = 0 then
		NUM = 1000 - NUM
	endif
	return

executecmd:
		select case  CMD
		case "f"
			pwmduty M1_PWM, NUM
			pwmduty M2_PWM, NUM
			if REVR <> 0 then
				low M1_DIR
				low M2_DIR
				let DIRNUM = DIRNUM - 2 * NUM
			else
				high M1_DIR
				high M2_DIR
				let DIRNUM = DIRNUM + 2 *NUM
			endif
			gosub dirlights
		case "l"
			pwmduty M1_PWM, NUM
			if REVR <> 0 then
				low M1_DIR
				let DIRNUM = DIRNUM - NUM
			else
				high M1_DIR
				let DIRNUM = DIRNUM + NUM
			endif
			gosub dirlights
		case "r"
			pwmduty M2_PWM, NUM
			if REVR <> 0 then
				low M2_DIR
				let DIRNUM = DIRNUM + NUM
			else
				high M2_DIR
				let DIRNUM = DIRNUM - NUM
			endif
			gosub dirlights
		case "t"
			pwmduty MT_PWM, NUM
			if REVR <> 0 then
				low MT_DIR
			else
				high MT_DIR
			endif
			gosub dirlights
		case "s"
			low		M1_DIR
			low		M2_DIR
			low		MT_DIR
			pwmout M1_PWM,150,0
			pwmout M2_PWM,150,0
			pwmout MT_PWM,150,0
			gosub dirlights
			high BLIGHT_LEFT
			high BLIGHT_RIGHT
		endselect
	bintoascii NUM, b1,b2,b3,b4,b5
	hserout 0,(b1)
	hserout 0,(b2)
	hserout 0,(b3)
	hserout 0,(b4)
	hserout 0,(b5)
	hserout 0,($0d)
	hserout 0,($0a)
	goto main		; loop back to start
	
readcmd:
	let TB1 = CMD_LOC
	do
		let TB2 = $FF
		hserin TB2
		if TB2 <> $ff then
			poke TB1, TB2
			TB1 = TB1 + 1
		endif
	loop until TB2 = $0D or TB1 > PARM_END or TB1 = CMD_LOC
	return
	
dirlights:
	if DIRNUM > 0 then
		high LIGHT_RIGHT
		low LIGHT_LEFT
	elseif DIRNUM < 0 then
		high LIGHT_LEFT
		low LIGHT_RIGHT
	else
		low LIGHT_LEFT
		low LIGHT_RIGHT
	endif
	low BLIGHT_LEFT
	low BLIGHT_RIGHT
	return

