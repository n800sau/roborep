'get cmd from PC to open a channel
'then all data from PC goes to the channel
'and data back goes to PC

'interrupt a px to communicate, then 

init:
	gosub fl_setup_serial

main:
	gosub fl_read_serial
	gosub fl_check_cmdbuf
	goto main

'include: ../baslib/funclib.bas

execute_cmd:
	select case @bptr
		case "o"
			'set output pin
			inc bptr
			'pin num
			let TW1_0 = @bptrinc
			let TW1_1 = @bptrinc
			'pin value
			let TW2_0 = @bptrinc
			let TW2_1 = @bptrinc
			output TW1
			if TW2 != 0 then
				high TW1
			else
				low TW1
			endif
		case "i"
			'read input pin
			inc bptr
			'pin num
			let TW1_0 = @bptrinc
			let TW1_1 = @bptrinc
			input TW1
			select case TW1
				case 0
					let TW2 = a.0
				case 1
					let TW2 = a.1
			endselect
			gosub fl_clear_reply_buf
			let bptr = SENDBUF_PTR
			let @bptrinc = "i"
			let @bptrinc = TW2_0
			let @bptrinc = TW2_1
			gosub fl_send_reply
		case "a"
			'read analog input pin
			inc bptr
			'pin num
			let TW1_0 = @bptrinc
			let TW1_1 = @bptrinc
			input TW1
			select case TW1
				case 0
					READADC10 0,TW2
				case 1
					READADC10 1,TW2
			endselect
			gosub fl_clear_reply_buf
			let bptr = SENDBUF_PTR
			let @bptrinc = "a"
			let @bptrinc = TW2_0
			let @bptrinc = TW2_1
			gosub fl_send_reply
	endselect
	return

