init:
	gosub setup_serial

main:
	gosub read_serial
	gosub check_cmdbuf
	goto main

'include: funclib.bas

