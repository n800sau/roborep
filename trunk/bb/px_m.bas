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

'include: funclib.bas

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
			if TW2 then
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
				case 0:
					let TW2 = a.0
			if TW2 then
				high TW1
			else
				low TW1
			endif
	endselect
	return

