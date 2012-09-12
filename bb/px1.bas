init:
	gosub setup_serial

main:
	gosub fl_read_serial
	gosub fl_check_cmdbuf
	goto main

execute_cmd:
	select case @bptr
		case 'z'
			'buzz
	endselect
	return

'include: funclib.bas

