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
symbol TW4 = w10
symbol TW4_0 = b18
symbol TW4_1 = b19

#serial command buffer
#serial command structure "X",bCMD,bLEN,bDATA*,bCRC
symbol CMDBUF_PTR = 100
symbol CMDBUF_END_PTR = CMDBUF_PTR + 79
symbol B_CUR_CMDBUF_PTR = CMDBUF_END_PTR + 1

#parsed command buffer
symbol UCMDBUF_PTR = B_CUR_CMDBUF_PTR + 1
symbol UCMDBUF_END_PTR = UCMDBUF_PTR + 1 + 2 + 2 + 2 + 2

#send back buffer
symbol SENDBUF_PTR = UCMDBUF_END_PTR + 1
symbol SENDBUF_END_PTR = SENDBUF_PTR + 30

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
	loop until @bptr = $0D
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
	let TB3 = UCMDBUF_PTR
	'read command byte
	poke TB3, @bptr
	inc bptr
	inc TB3
	'parse 4 parameters
	for TB2 = 1 TO 4
		gosub parse_num
		poke TB3, WORD TW1
		let TB3 = TB3 + 2
		'skip until next digit
		do
			inc bptr
		loop until (@bptr >= "0" and @bptr <= "9") or @bptr = $0D
	next
	return

parse_num:
	'parameters @bptr - pointer to the beginning of ascii source string
	'returns value in TW1
	'uses TB1
	symbol NEG = TB1
	symbol NUM = TW1
	let NUM = 0
	if @bptr = '-' then
		let NEG = 1
		inc bptr
	else
		let NEG = 0
	endif
	do while @bptr >= "0" and @bptr <= "9"
		let NUM = NUM * 10 + @bptr - "0"
		inc bptr
	loop
	if NEG <> 0 then
		let NUM = -NUM
	endif
	return

execute_cmd:
	let bptr = UCMDBUF_PTR
	select case @bptr
		case "e"
			'echo command
			hserout 0,(@bptrinc)
			hserout 0,(":")
			for TB1 = 1 to 4
				let TW1_1 = @bptrinc
				let TW1_2 = @bptrinc
				bintoascii TW1, TW2_1, TW2_2, TW3_1, TW3_2, TW4_1
				hserout 0,(TW2_1)
				hserout 0,(TW2_2)
				hserout 0,(TW3_1)
				hserout 0,(TW3_2)
				hserout 0,(TW4_1)
				if TB1 < 4 then
					hserout 0,(",")
				endif
			next
			hserout 0,($0d)
			hserout 0,($0a)
	endselect
	return
