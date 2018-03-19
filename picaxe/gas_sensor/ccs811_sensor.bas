symbol RESET_PIN = C.1

symbol CCS811_ADDRESS = 0x5A * 2

symbol CCS811_STATUS = 0x00
symbol CCS811_MEAS_MODE = 0x01
symbol CCS811_ALG_RESULT_DATA = 0x02
symbol CCS811_RAW_DATA = 0x03
symbol CCS811_ENV_DATA = 0x05
symbol CCS811_NTC = 0x06
symbol CCS811_THRESHOLDS = 0x10
symbol CCS811_BASELINE = 0x11
symbol CCS811_HW_ID = 0x20
symbol CCS811_HW_VERSION = 0x21
symbol CCS811_FW_BOOT_VERSION = 0x23
symbol CCS811_FW_APP_VERSION = 0x24
symbol CCS811_ERROR_ID = 0xE0
symbol CCS811_SW_RESET = 0xFF

symbol CCS811_BOOTLOADER_APP_ERASE = 0xF1
symbol CCS811_BOOTLOADER_APP_DATA = 0xF2
symbol CCS811_BOOTLOADER_APP_VERIFY = 0xF3
symbol CCS811_BOOTLOADER_APP_START = 0xF4

symbol CCS811_DRIVE_MODE_IDLE = 0x00
symbol CCS811_DRIVE_MODE_1SEC = 0x10
symbol CCS811_DRIVE_MODE_10SEC = 0x20
symbol CCS811_DRIVE_MODE_60SEC = 0x30
symbol CCS811_DRIVE_MODE_250MS = 0x40

symbol CCS811_HW_ID_CODE = $81

	hi2csetup i2cmaster, CCS811_ADDRESS, i2cslow, i2cbyte
	sertxd (cr,lf,"START PROGRAM")

main:
	low RESET_PIN
	pause 200
	high RESET_PIN
	pause 500
'	let b0 = 0x11
'	let b1 = 0xE5
'	let b2 = 0x72
'	let b4 = 0x8A
'	hi2cout CCS811_SW_RESET, (b1, b2, b3, b4)
'	pause 200
	hi2cin CCS811_HW_ID, (b0)
	sertxd (cr,lf,"HW_ID:")
	gosub printhex

	gosub is_ok

	if b2 <> 0 then
'		let b0 = 0
'		hi2cout CCS811_BOOTLOADER_APP_START, (b0)
'		pause 100
'		let b0 = CCS811_DRIVE_MODE_1SEC
'		hi2cout CCS811_MEAS_MODE, (b0)
		gosub is_ok
		if b2 <> 0 then
			do while b2 <> 3
				pause 1000
				gosub is_ok
			loop
			hi2cin CCS811_ALG_RESULT_DATA, (b1, b2, b3, b4, b5, b6, b7, b8)
			sertxd (cr,lf,"DATA:")
			let b0 = b1
			gosub printhex
			let b0 = b2
			gosub printhex
			let b0 = b3
			gosub printhex
			let b0 = b4
			gosub printhex
			let b0 = b5
			gosub printhex
			let b0 = b6
			gosub printhex
			let b0 = b7
			gosub printhex
			let b0 = b8
			gosub printhex
		endif
	endif

	pause 3000
	goto main

is_ok:
	; return 1 or 0 in b2
	hi2cin CCS811_STATUS, (b0)
	sertxd (cr,lf,"STATUS:")
	gosub printhex
	let b1 = b0 & 1
	if b1 <> 0 then
		hi2cin CCS811_ERROR_ID, (b0)
		sertxd (cr,lf,"ERROR:")
		gosub printhex
		let b2 = 0
	else
		let b1 = b0 & 0x10
		if b1 <> 0 then
			sertxd (cr,lf,"APP VALID")
			let b2 = 1
		endif
		let b1 = b0 & 0x80
		if b1 <> 0 then
			sertxd (cr,lf,"FW MODE")
			let b2 = 2
		endif
		let b1 = b0 & 0x8
		if b1 <> 0 then
			sertxd (cr,lf,"DATA READY")
			let b2 = 3
		endif
	endif
	return

printhex:
	; b0 - input
	; b1 in use
	;
	; Print in hex too
	;
	b1 = b0 / 16 + "0"            ;get top  nybble, convert to ASCII
	if b1 > "9" then
	      b1 = b1 + 7             ;convert 9-15 into A-F
	endif
	sertxd(" 0x", b1)                    ;print it

	b1 = b0 & %00001111 + "0"     ;get bottom nybble, convert to ASCII
	if b1 > "9" then
	      b1 = b1 + 7             ;convert 9-15 into A-F
	endif
	sertxd(b1)                    ;print it
	return