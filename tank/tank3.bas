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

'include: ../baslib/funclib.bas

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
	fvrsetup FVR2048
	'set FVR as ADC Vref+, 0V Vref-
	adcconfig %011
	let TW4 = 0

main:
	gosub fl_read_serial
	gosub fl_check_cmdbuf
	gosub m_send_reply
	debug
	pause 1
	goto main

m_send_reply:
	'TB1 - cmd
	'TW1 - num
	gosub fl_clear_reply_buf
	let bptr = SENDBUF_PTR
	let @bptrinc = TB1
	let @bptrinc = TW1_0
	let @bptrinc = TW1_1
	gosub fl_send_reply

execute_cmd:
	let TB1 = @bptrinc
	let TW1_0 = @bptrinc
	let TW1_1 = @bptrinc
	select case TB1
		case "f"
			pwmduty M1_PWM, TW1
			pwmduty M2_PWM, TW1
			if TW1 < 0 then
				low M1_DIR
				low M2_DIR
			else
				high M1_DIR
				high M2_DIR
			endif
			gosub m_send_reply
		case "l"
			pwmduty M1_PWM, TW1
			if TW1 < 0 then
				low M1_DIR
			else
				high M1_DIR
			endif
			gosub m_send_reply
		case "r"
			pwmduty M2_PWM, TW1
			if TW1 < 0 then
				low M2_DIR
			else
				high M2_DIR
			endif
			gosub m_send_reply
		case "t"
			pwmduty MT_PWM, TW1
			if TW1 < 0 then
				low MT_DIR
			else
				high MT_DIR
			endif
			gosub m_send_reply
		case "s"
			low		M1_DIR
			low		M2_DIR
			low		MT_DIR
			pwmout M1_PWM,150,0
			pwmout M2_PWM,150,0
			pwmout MT_PWM,150,0
			gosub m_send_reply
		case "<"
			readadc10 V_BATT1, TW1
			let TW1 = TW1 * 2048 / 1023
			gosub m_send_reply
		case ">"
			readadc10 V_BATT2, TW1
			let TW1 = TW1 * 2048 / 1023
			gosub m_send_reply
		case "{"
			readadc10 V_SW1, TW1
			let TW1 = TW1 * 2048 / 1023
			gosub m_send_reply
		case "}"
			readadc10 V_SW2, TW1
			let TW1 = TW1 * 2048 / 1023
			gosub m_send_reply
	endselect
	return
