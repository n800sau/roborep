'current command to execute
symbol CMD = b0
symbol NUM1 = w1
symbol NUM2 = w2
symbol NUM3 = w3
symbol NUM4 = w4

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

#serial command buffer
#serial command structure "X",bCMD,bLEN,bDATA*,bCRC
symbol CMDBUF_PTR = 100
symbol CMDBUF_END_PTR = 179
symbol B_CUR_CMDBUF_PTR = 121

setfreq m32

init:
	hsersetup B9600_32 ,%00			; baud 9600 at 32MHz
	gosub reset_cmdbuf

main:
	gosub read_serial
	gosub check_cmdbuf
	goto main

read_serial:
	do
		peek B_CUR_CMDBUF_PTR, bptr
		if bptr = CMDBUF_PTR then
			'search for 'X'
			let @bptr = $FF
			hserin @bptr
			if @bptr <> "X" then
				'nothing to read
				return
			endif
			'command begins
		endif
		let bptr = bptr + 1
		if bptr > CMDBUF_END_PTR then
			'overloading, complain by some means
			gosub scream_error
			gosub reset_cmdbuf
			return
		endif
		let @bptr = $FF
		hserin @bptr
		if @bptr = $FF then
			'nothing to read
			return
		endif
		poke B_CUR_CMDBUF_PTR, bptr
		'command ends with $0D
	until @bptr = $0D
	return

reset_cmdbuf:
	'reset cmdbuf pointer
	poke B_CUR_CMDBUF_PTR, CMDBUF_PTR
	'no command sign
	poke CMDBUF_PTR, $00
	let CMD = 0
	return

scream_error:
	'here it should be some way to show error
	return

check_cmdbuf:
	peek B_CUR_CMDBUF_PTR, bptr
	if @bptr = $0D then
		'parse cmdbuf
		gosub parse_cmdbuf
		'execute_cmd
		gosub execute_cmd
		'reset command buffer for a new command
		gosub reset_cmdbuf
	endif
	return

parse_cmdbuf:
	'skip X
	let bptr = CMDBUF_PTR + 1
	'read command byte
	let CMD = @bptr
	inc bptr
	'read parameters
	'number
	let TW1 = 0
	do
	until bptr

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
