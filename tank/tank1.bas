symbol M1_PWM = c.5
symbol M1_DIR = c.4
symbol M2_PWM = c.3
symbol M2_DIR = c.2
symbol CMD_LOC = 128
symbol PARM_START= CMD_LOC + 1
symbol PARM_END = 138
symbol CMD = b1
symbol NUM = w3
'temporary vars
symbol TB1 = b8
symbol TB2 = b9
symbol TB3 = b10
symbol TB4 = b11
symbol TW1 = w6
symbol TW2 = w7

init:	
		table 0,("Hello World")
		hsersetup B9600_4 ,%00			; baud 19200 at 4MHz
		pwmout M1_PWM,150,0
		pwmout M2_PWM,150,0
'		output M1_PWM
'		output M2_PWM
'		low		M1_PWM
'		low 		M2_PWM
		output M1_DIR
		output M2_DIR
		low		M1_DIR
		low		M2_DIR
		
		'high 400 -  0
		'low 200-1023
'		pwmduty       M1_PWM, 1023
'		pwmduty M2_PWM,200


main:
	gosub readcmd
	gosub parsecmd
	gosub executecmd
	goto main
		
parsecmd:
	peek CMD_LOC, CMD
	let NUM = 0
	let TB3 = 1
	for TB1 = PARM_START to PARM_END
		peek TB1, TB2
		if TB1 = PARM_START and TB2 = "-" then
			let TB3 = -1
		else
			if TB2 <"0" or TB2 > "9" then
				exit
			endif
			let NUM = NUM * 10 + TB2 - "0"
		endif
	next
	let NUM = NUM * TB3
	return

executecmd:
		select case  CMD
		case "l"
'			pwmduty M1_PWM, NUM
'			high M1_PWM
			pause 100
'			low M1_PWM
		case "r"
'			pwmduty M2_PWM, NUM
'			high M2_PWM
			pause 100
'			low M2_PWM
		case 's'
			if M1_DIR
			pwmduty M2_PWM, 
		endselect
	hserout 0,(b1)
	bintoascii NUM, b1,b2,b3,b4,b5
	hserout 0,(b1)
	hserout 0,(b2)
	hserout 0,(b3)
	hserout 0,(b4)
	hserout 0,(b5)
	hserout 0,($0d)
	hserout 0,($0a)
'	pause 1000		; pause 1 s
'	low M1_DIR
'	high M2_DIR
'	pause 1000		; pause 1 s
'		high	M1_DIR
'		low 	M2_DIR
'	pause 1000
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
	loop until TB2 = $0D or TB1 > PARM_END
	return
	