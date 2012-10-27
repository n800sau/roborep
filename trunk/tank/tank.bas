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

symbol U_ECHO = b.1
symbol U_TRIG = b.0

'b.6,b.5 free

'include: ../baslib/tempvar.bas

init:
	gosub fl_setup_serial
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
	'set FVR as ADC Vref+, 0V Vref-
	adcconfig %011
'	adcconfig %000
	'set FVR as 4.096V
	fvrsetup FVR4096
'	let TW4 = 0

main:
	gosub u_distance
	gosub fl_read_serial
	gosub fl_check_cmdbuf
'	gosub m_send_reply
'	debug
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

execute_cmd:
	let TB1 = @bptrinc
	let TW1_0 = @bptrinc
	let TW1_1 = @bptrinc
	select case TB1
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
			let TW1 = UDISTANCE_PTR
			peek TW1, WORD TW2
			let bptr = SENDBUF_PTR
			let @bptrinc = TB1
			let @bptrinc = TW2_0
			let @bptr = TW2_1
			gosub m_send_reply
	endselect
	return

u_distance:
	'get distance
	'uses TW1, bptr
	low U_TRIG
	setint %00000000,%00000000 		' disable interrupts
	pulsout U_TRIG, 1				' Send a pulse to start the ranging
	pulsin U_ECHO, 1, TW1			' Recieve timed pulse from SRF05/04
	setint %00010000,%00010000		' enable interrupt
	TW1 = TW1 * 10 / 58				' Calculate distance
	if TW1 > 0 then
		let bptr = UDISTANCE_PTR
		let @bptrinc = TW1_0
		let @bptr = TW1_1
	endif
	return

interrupt:
	return
