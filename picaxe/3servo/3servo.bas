symbol tmp=b0
symbol CMD=b1
symbol ARG=b2
symbol I2C_ADDR=b3

symbol MIDDLE=145
symbol MINPOS=75
symbol MAXPOS=225
symbol POS=w5

symbol BOTTOM=B.0
symbol TILT=B.1
symbol PAN=B.2
symbol MIN_I2C=3

init:
	servo BOTTOM, MIDDLE
	servo TILT, MIDDLE
	servo PAN, MIDDLE
	hsersetup B115200_4, 0
	I2C_ADDR = MIN_I2C

main:
	; get first byte
	CMD = 0
	hserin CMD
	if CMD<>0 then
		; get second byte
		ARG = 0
		do
			hserin ARG
		loop while ARG=0
		hserout 0, (CMD, "(", ARG, ")", 10, 13)
		select case CMD
			case "b"
				sertxd("Set bottom to:", #ARG, 13, 10)
				servopos BOTTOM, ARG
			case "t"
				sertxd("Set tilt to:", #ARG, 13, 10)
				servopos TILT, ARG
			case "p"
				sertxd("Set pan to:", #ARG, 13, 10)
				servopos PAN, ARG
			else
				sertxd("Unknown command:", CMD, 13, 10)
		endselect
		; flush fifoes
		do
			POS=0
			hserin POS
		loop until POS=0
	else
		; no command entered
		; read i2c
		hi2csetup I2CMASTER, I2C_ADDR, i2cslow, i2cbyte
		tmp = 0
		hi2cin $75, (tmp)
		sertxd("At ", #I2C_ADDR, " : ", #tmp, 13, 10)
		inc I2C_ADDR
		if I2C_ADDR > 127 then
			I2C_ADDR = MIN_I2C
		endif
	endif
	goto main
