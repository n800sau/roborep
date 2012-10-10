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
	'set FVR as 2.048V
	fvrsetup FVR4096
'	fvrsetup FVR2048
	'set FVR as ADC Vref+, 0V Vref-
'	adcconfig %011
	adcconfig %000
'	let TW4 = 0

main:
'	let TW4 = TW4+1
'	let TB1 = ">"
'	let TW2 = TW4
	gosub fl_read_serial
	gosub fl_check_cmdbuf
'	gosub m_send_reply
'	debug
'	pause 1000
	goto main

'include: ../baslib/funclib.bas

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
		case "<"
			readadc10 V_BATT1, TW2
'			let TW2 = TW2 * 2048 / 1023
			gosub m_send_reply
		case ">"
			readadc10 V_BATT2, TW2
'			let TW2 = TW2 * 2048 / 1023
			gosub m_send_reply
		case "{"
			readadc10 V_SW1, TW2
'			let TW2 = TW2 * 2048 / 1023
			gosub m_send_reply
		case "}"
			readadc10 V_SW2, TW2
'			let TW2 = TW2 * 2048 / 1023
			gosub m_send_reply
	endselect
	return
