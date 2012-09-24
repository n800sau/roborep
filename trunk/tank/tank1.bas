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


symbol TESTLIGHTS_TASK_LOC = PARM_END + 2 'current testlights action
symbol TESTLIGHTS_TASK_TIME_LOC = TESTLIGHTS_TASK_LOC + 1 ' word - time of the next testlights action

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
symbol COUNTER = w9

symbol DIRNUM = w10

'ADXL345
symbol ACCEL_I2C_ADDR = 0x53
symbol accel_power = 0x2D
symbol accel_dataformat = 0x31
symbol accel_x = 0x32
symbol accel_y = 0x34
symbol accel_z = 0x36
'$50 to $7E
symbol accel_x_loc = 150
symbol accel_y_loc = 152
symbol accel_z_loc = 154

'BMP085
symbol BMP_I2C_ADDR = $68

setfreq m32

init:	
		table 0,("Hello World")
		poke IDLE_TASK_LOC, 0
		hsersetup B9600_32 ,%00			; baud 19200 at 4MHz
		hi2csetup i2cmaster, ACCEL_I2C_ADDR, i2cfast_32, i2cbyte
'		hi2cout accel_power, (0)
'		hi2cout accel_power, (16)
		hi2cout accel_power, (8)
		let COUNTER = 0
		poke TESTLIGHTS_TASK_LOC, 0
		pwmout M1_PWM,150,0
		pwmout M2_PWM,150,0
		pwmout MT_PWM,150,0
'		output M1_PWM
'		output M2_PWM
'		low		M1_PWM
'		low 		M2_PWM
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
	if TB1 > CMD_LOC then
		gosub parsecmd
		gosub executecmd
		gosub idletask
	endif
	gosub get_accel_data
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
		case "z"
			peek IDLE_TASK_LOC, TB1
'			setbit TB1, LIGHT_TASK
			poke IDLE_TASK_LOC, TB1
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
	
tick_test_lights:
	peek TESTLIGHTS_TASK_LOC, TB1
	if TB1 = 0 then
	  poke TESTLIGHTS_TASK_TIME_LOC, Time
	endif
    peek TESTLIGHTS_TASK_TIME_LOC, TW1
    if Time > TW1 then
      let TW1 = Time + 1
      'next tick happened after 1 second
	  poke TESTLIGHTS_TASK_TIME_LOC, TW1
	  'check what should be done next
	  inc TB1
	  select case TB1
		case 1
		  high	LIGHT_LEFT
		case 2
		  low		LIGHT_LEFT
		  high	LIGHT_RIGHT
		case 3
		  low		LIGHT_RIGHT
		  high	BLIGHT_LEFT
		case 4
		  low	BLIGHT_LEFT
		  high	BLIGHT_RIGHT
		case 5
		  low	BLIGHT_RIGHT
		  peek IDLE_TASK_LOC, TB1
'		  setbit TB1, LIGHT_TASK
		  poke IDLE_TASK_LOC, TB1
'		  unsetbit TB1, LIGHT_TASK
		  let TB1 = 0
	  endselect
  	  poke TESTLIGHTS_TASK_LOC, TB1
	endif
	return

idletask:
	peek IDLE_TASK_LOC, TB1
'	if TB1 bit LIGHT_TASK set then
	  gosub tick_test_lights
'	endif
	return

get_accel_data:
	debug
	hi2csetup i2cmaster, ACCEL_I2C_ADDR, i2cfast_32, i2cbyte
	let bptr=accel_x_loc
	for TB1=1 to 6
		let @bptrInc = 0
	next
	let bptr = accel_x_loc
	hi2cin  accel_x, (@bptrInc, @bptrInc, @bptrInc, @bptrInc, @bptrInc, @bptrInc)
	let bptr = accel_x_loc
	bintoascii TW1, b1,b2,b3,b4,b5
	hserout 0,(b1, b2, b3, b4, b5, $0d, $0a)
	return

get_bmp085:

	hi2csetup i2cmaster, BMP_I2C_ADDR, i2cfast_32, i2cbyte
	return
