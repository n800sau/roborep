symbol M1_PWM = c.5
symbol M1_DIR = c.4
symbol M2_PWM = c.3
symbol M2_DIR = c.2
'symbol MT_PWM = b.1
'symbol MT_DIR = b.0

symbol V_BATT1 = b.2
symbol V_SW1 = b.3
symbol V_BATT2 = b.4
symbol V_SW2 = b.5

symbol U_ECHO = b.1
symbol U_TRIG = b.0

'b.6,b.5 free

'include: ../baslib/tempvar.bas

init:
	gosub fl_setup_serial
	pwmout M1_PWM,150,0
	pwmout M2_PWM,150,0
'	pwmout MT_PWM,150,0
	output M1_DIR
	output M2_DIR
'	output MT_DIR
	low		M1_DIR
	low		M2_DIR
'	low		MT_DIR
	input	V_BATT1
	input	V_SW1
	input	V_BATT2
	input	V_SW2
	'set FVR as ADC Vref+, 0V Vref-
	adcconfig %011
'	adcconfig %000
	'set FVR as 4.096V
	fvrsetup FVR4096
'	let TW4 = 0

main:
	gosub fl_read_serial
	gosub fl_check_cmdbuf
'	gosub m_send_reply
'	gosub u_distance
'	debug
'	high U_TRIG
'	pause 10
'	low U_TRIG
'	pause 1000
	goto main

'include: ../baslib/funclib.bas

symbol UDISTANCE_PTR = USER_MEM

m_send_reply:
	'TB1 - cmd
	'TW2 - num
	gosub fl_clear_reply_buf
	let bptr = SENDBUF_PTR
	let @bptrinc = TB1
	let @bptrinc = TW2_0
	let @bptrinc = TW2_1
	gosub fl_send_reply
	return

corrMvars:
	if TW1 < 0 then
		let TW2 = 1
	else
		let TW2 = 0
	endif
	if TW1 > 1000 then
		TW1 = 1000
	endif
	if TW2 = 0 then
		TW1 = 1000 - TW1
	endif
	return

execute_cmd:
	let TB1 = @bptrinc
	let TW1_0 = @bptrinc
	let TW1_1 = @bptrinc
	select case TB1
		case "f"
			gosub corrMvars
			pwmduty M1_PWM, TW1
			pwmduty M2_PWM, TW1
			if TW2 <> 0 then
				low M1_DIR
				low M2_DIR
			else
				high M1_DIR
				high M2_DIR
			endif
		case "l"
			gosub corrMvars
			pwmduty M1_PWM, TW1
			if TW2 <> 0 then
				low M1_DIR
			else
				high M1_DIR
			endif
		case "r"
			gosub corrMvars
			pwmduty M2_PWM, TW1
			if TW2 <> 0 then
				low M2_DIR
			else
				high M2_DIR
			endif
		case "s"
			low		M1_DIR
			low		M2_DIR
			pwmout M1_PWM,150,0
			pwmout M2_PWM,150,0
		case "V"
			let bptr = SENDBUF_PTR
			let @bptrinc = TB1

			readadc10 V_BATT1, TW2
			'vd/9.9*(26.8+9.9) = v
'			let TW2 = 4 * 367 / 99 * TW2
			let @bptrinc = TW2_0
			let @bptrinc = TW2_1

			readadc10 V_BATT2, TW2
'			let TW2 = 4 * 366 / 98 * TW2
			let @bptrinc = TW2_0
			let @bptrinc = TW2_1

			readadc10 V_SW1, TW2
'			let TW2 = 4 * 365 / 97 * TW2
			let @bptrinc = TW2_0
			let @bptrinc = TW2_1

			readadc10 V_SW2, TW2
'			let TW2 = 4 * 3638 / 978 * TW2
			let @bptrinc = TW2_0
			let @bptrinc = TW2_1

			gosub fl_send_reply
		case "U"
			gosub u_distance
'			let TW1 = UDISTANCE_PTR
'			peek TW1, WORD TW2
			let bptr = SENDBUF_PTR
			let @bptrinc = TB1
			let @bptrinc = TW2_0
			let @bptr = TW2_1
			gosub m_send_reply
	endselect
	return

u_distance:
	'get distance
	'uses TW1, TW2
	'return TW2
	let TW1 = UDISTANCE_PTR
	let TW2 = 1
	poke TW1, WORD TW2
	low U_TRIG
	setint %00000000,%00000000 		' disable interrupts
	pulsout U_TRIG, 1				' Send a pulse to start the ranging
	pulsin U_ECHO, 1, TW2			' Recieve timed pulse from SRF05/04
	setint %00010000,%00010000		' enable interrupt
	let TW2 = TW2 * 25 / 580				' Calculate distance
'	if TW2 > 0 then
'		poke TW1, WORD TW2
'	endif
	return

interrupt:
	return
