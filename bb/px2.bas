symbol P_FWD_LEFT c.1
symbol P_FWD_RIGHT c.2
symbol P_BACK_LEFT c.3
symbol P_BACK_RIGHT c.4

symbol P_LIGHT c.5

symbol P_CLEAR 0
symbol P_CLOSE 100
symbol P_TOO_CLOSE 200
symbol P_BUMP 250

init:
	gosub setup_serial

main:
	gosub read_serial
	gosub check_cmdbuf
	goto main

'include: funclib.bas

read_perimeter:
	'light on
	high P_LIGHT
	gosub fl_clear_reply_buf
	let TB1 = FWD_LEFT
	let TB2 = 'l'
	gosub send_ir
	let TB1 = FWD_RIGHT
	let TB2 = 'r'
	gosub send_ir
	let TB1 = BACK_LEFT
	let TB2 = 'L'
	gosub send_ir
	let TB1 = BACK_RIGHT
	let TB2 = 'R'
	gosub send_ir
	'light off
	low P_LIGHT
	return

send_ir:
	'TB1 - ir
	'TB2 - type
	poke SENDBUF_PTR, TB2
	readadc10 TB1, TW1
	gosub estimate
	let TW2 = TB3
	poke SENDBUF_PTR, "l"
	poke SENDBUF_PTR+1, TW1_1
	poke SENDBUF_PTR+2, TW1_2
	poke SENDBUF_PTR+1, TW2_1
	poke SENDBUF_PTR+2, TW2_2
	gosub fl_send_reply

estimate:
	'TW1 - val
	'returns in TB3
	if TW1 > P_BUMP then
		let TB3 = P_BUMP
	elif TW1 > P_TOO_CLOSE - 20 and TW1 < P_TOO_CLOSE + 20 then
		let TB3 = P_TOO_CLOSE
	elif TW1 > P_CLOSE - 20 and TW1 < P_CLOSE + 20 then
		let TB3 = P_CLOSE
	else
		let TB3 = P_CLEAR

execute_cmd:
	select case @bptr
		case 'g'
			gosub read_perimeter
	endselect
	return

