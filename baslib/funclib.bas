'serial command buffer
'NO:serial command structure "X",bCMD,bLEN,bDATA*,bCRC
'YES:serial command structure "X",bCMD,:,NUM1?,NUM2?,NUM3?,NUM4?,$0D
symbol CMDBUF_PTR = 100
symbol CMDBUF_END_PTR = CMDBUF_PTR + 79
symbol B_CUR_CMDBUF_PTR = CMDBUF_END_PTR + 1

'parsed command buffer
symbol UCMDBUF_PTR = B_CUR_CMDBUF_PTR + 1
symbol UCMDBUF_END_PTR = UCMDBUF_PTR + 9

'send back buffer
symbol SENDBUF_PTR = UCMDBUF_END_PTR + 1
symbol SENDBUF_END_PTR = SENDBUF_PTR + 30

fl_setup_serial:
	setfreq m16
	hsersetup B19200_16 ,%00			; baud 19200 at 16MHz
	gosub fl_reset_cmdbuf
	return

fl_scream_error:
	'here it should be some way to show error
	return

fl_read_serial:
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
			gosub fl_scream_error
			gosub fl_reset_cmdbuf
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

fl_reset_cmdbuf:
	'reset cmdbuf pointer
	poke B_CUR_CMDBUF_PTR, CMDBUF_PTR
	'no command sign
	poke CMDBUF_PTR, $00
	return

fl_check_cmdbuf:
	peek B_CUR_CMDBUF_PTR, bptr
	if @bptr = $0D then
		'parse cmdbuf
		gosub fl_parse_cmdbuf
		'execute_cmd
		gosub fl_execute_cmd
		'reset command buffer for a new command
		gosub fl_reset_cmdbuf
	endif
	return

fl_parse_cmdbuf:
	'skip X
	let bptr = CMDBUF_PTR + 1
	let TB3 = UCMDBUF_PTR
	'read command byte
	poke TB3, @bptr
	inc bptr
	'skip :
	inc bptr
	inc TB3
	'parse 4 parameters
	for TB2 = 1 TO 4
		gosub fl_parse_num
		poke TB3, WORD TW1
		let TB3 = TB3 + 2
		'skip until next digit
		do
			inc bptr
		loop until @bptr >= "0" and @bptr <= "9" or @bptr = $0D
	next
	return

fl_parse_num:
	'parameters @bptr - pointer to the beginning of ascii source string
	'returns value in TW1
	'uses TB1
	let TW1 = 0
	if @bptr = "-" then
		let TB1 = 1
		inc bptr
	else
		let TB1 = 0
	endif
	do while @bptr >= "0" and @bptr <= "9"
		let TW1 = TW1 * 10 + @bptr - "0"
		inc bptr
	loop
	if TB1 <> 0 then
		let TW1 = -TW1
	endif
	return

fl_execute_cmd:
	let bptr = UCMDBUF_PTR
	select case @bptr
		case "e"
			'echo command
			let TB1 = "e"
			inc bptr
			peek bptr, WORD TW1
			inc bptr
			inc bptr
			peek bptr, WORD TW2
			inc bptr
			inc bptr
			peek bptr, WORD TW3
			inc bptr
			inc bptr
			peek bptr, WORD TW4
			inc bptr
			inc bptr
			gosub fl_fill_reply_buf
			gosub fl_send_reply
		else
			gosub execute_cmd
	endselect
	return

fl_clear_reply_buf:
	'uses TW1
	for TW1 = SENDBUF_PTR to SENDBUF_END_PTR
		poke TW1, 0
	next
	return

fl_fill_reply_buf:
	'TB1 - type
	'TW1 - arg1
	'TW2 - arg2
	'TW3 - arg3
	'TW4 - arg4
	let bptr = SENDBUF_PTR
	let @bptrinc = TB1
	let @bptrinc = TW1_0
	let @bptrinc = TW1_1
	let @bptrinc = TW2_0
	let @bptrinc = TW2_1
	let @bptrinc = TW3_0
	let @bptrinc = TW3_1
	let @bptrinc = TW4_0
	let @bptrinc = TW4_1
	return

fl_send_reply:
	'uses TB1, TB2, TW1, TW2, TW3
	let bptr = SENDBUF_PTR
	hserout 0,(@bptrinc)
	hserout 0,(":")
	for TB2 = 1 to 4
		let TW1_0 = @bptrinc
		let TW1_1 = @bptrinc
		bintoascii TW1, TB1, TW2_0, TW2_1, TW3_0, TW3_1
		hserout 0,(TB1)
		hserout 0,(TW2_0)
		hserout 0,(TW2_1)
		hserout 0,(TW3_0)
		hserout 0,(TW3_1)
		if TB2 < 4 then
			hserout 0,(",")
		endif
	next
'	hserout 0,($0d)
	hserout 0,($0a)
	return
