init:
	gosub fl_setup_serial

main:
	gosub fl_read_serial
	gosub fl_check_cmdbuf
	goto main

execute_cmd:
	return

'include: ../baslib/funclib.bas

