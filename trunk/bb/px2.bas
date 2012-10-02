symbol P_FWD_LEFT = c.1
symbol P_FWD_RIGHT = c.2
symbol P_BACK_LEFT = c.3
symbol P_BACK_RIGHT = c.4

symbol P_LIGHT = c.5

symbol P_CLEAR = 0
symbol P_CLOSE = 100
symbol P_TOO_CLOSE = 200
symbol P_BUMP = 250

init:
	gosub fl_setup_serial

main:
	gosub fl_read_serial
	gosub fl_check_cmdbuf
	goto main

'include: ../baslib/funclib.bas

read_perimeter:
	'light on
	high P_LIGHT
	gosub fl_clear_reply_buf
	let TB1 = P_FWD_LEFT
	let TB2 = "l"
	gosub send_ir
	let TB1 = P_FWD_RIGHT
	let TB2 = "r"
	gosub send_ir
	let TB1 = P_BACK_LEFT
	let TB2 = "L"
	gosub send_ir
	let TB1 = P_BACK_RIGHT
	let TB2 = "R"
	gosub send_ir
	'light off
	low P_LIGHT
	return

send_ir:
	'TB1 - ir
	'TB2 - type
	let bptr = SENDBUF_PTR
	poke @bptrinc, TB2
	readadc10 TB1, TW1
	gosub estimate
	let TW2 = TB3
	poke @bptrinc, TW1_0
	poke @bptrinc, TW1_1
	poke @bptrinc, TW2_0
	poke @bptrinc, TW2_1
	gosub fl_send_reply

estimate:
	'TW1 - val
	'uses TW2 and TW3
	'returns in TB3
	let TW2 = TW1 + 20
	let TW3 = TW1 - 20
	if TW1 > P_BUMP then
		let TB3 = P_BUMP
'	elseif TW2 > P_TOO_CLOSE and TW3 < P_TOO_CLOSE then
		let TB3 = P_TOO_CLOSE
	elseif TW2 > P_CLOSE and TW3 < P_CLOSE then
		let TB3 = P_CLOSE
	else
		let TB3 = P_CLEAR
	endif

execute_cmd:
	select case @bptr
		case "g"
			gosub read_perimeter
	endselect
	return

