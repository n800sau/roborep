symbol sclk = C.5			; clock (output pin)
symbol sdata = C.7			; data (output pin for shiftout)
symbol serdata_pin = C.6	; data (input pin for shiftin, note input7)
symbol serdata = pinC.6		; data (input pin for shiftin, note input7)
symbol adxl_cs = C.4		; select adxl345
symbol rf_cs = C.2			; select nrf24l01+
symbol rf_ce = C.3			; ce nrf24l01+
symbol rf_irq = C.1			; irq nrf24l01+
symbol adxl_int1 = B.7		; adxl345 int1
symbol adxl_int2 = B.6		; adxl345 int2

symbol counter = b0			; variable used during loop
symbol mask = b1			; bit masking variable
symbol var_in = b2			; data variable used durig shiftin
symbol var_out = b3			; data variable used during shiftout
symbol reg = b5				; register address
symbol regval = b6			; register value byte
symbol regword = w3			; register value byte
symbol regword.0 = b6		; register value byte
symbol regword.1 = b7		; register value byte
symbol val16.0 = b10			; 16 bit value
symbol val16.1 = b11			; 16 bit value
symbol val16 = w5			; 16 bit value
symbol vlabel = b12			; label for output
symbol i = b20				; loop var
'symbol j = w11				; loop var
Symbol Nref = w12			; Word variable for the calibadc10 reading
symbol bits = 8				; number of bits
symbol MSBvalue = 128			; MSBvalue =128 for 8 bits, 512 for 10 bits, 2048 for 12 bits)
symbol DATA_FORMAT = 0x31	;
symbol POWER_CTL = 0x2d		; Power Control Register

'Memory Map
symbol CONFIG      = 0x00
symbol EN_AA       = 0x01
symbol EN_RXADDR   = 0x02
symbol SETUP_AW    = 0x03
symbol SETUP_RETR  = 0x04
symbol RF_CH       = 0x05
symbol RF_SETUP    = 0x06
symbol STATUS      = 0x07
symbol OBSERVE_TX  = 0x08
symbol CD          = 0x09
symbol RX_ADDR_P0  = 0x0A
symbol RX_ADDR_P1  = 0x0B
symbol RX_ADDR_P2  = 0x0C
symbol RX_ADDR_P3  = 0x0D
symbol RX_ADDR_P4  = 0x0E
symbol RX_ADDR_P5  = 0x0F
symbol TX_ADDR     = 0x10
symbol RX_PW_P0    = 0x11
symbol RX_PW_P1    = 0x12
symbol RX_PW_P2    = 0x13
symbol RX_PW_P3    = 0x14
symbol RX_PW_P4    = 0x15
symbol RX_PW_P5    = 0x16
symbol FIFO_STATUS = 0x17
symbol DYNPD       = 0x1C
symbol FEATURE     = 0x1D


input serdata_pin
output sdata
high adxl_cs

high rf_cs
low rf_ce
input rf_irq

input adxl_int1
input adxl_int2


'Put the ADXL345 into +/- 2G range by writing the value 0x00 to the DATA_FORMAT register
let reg = DATA_FORMAT
let regval = 0x08
'let regval = 0x05
gosub writeRegister_adxl
let reg = 0x1e
let regval = -50
'gosub writeRegister_adxl
let reg = 0x1f
let regval = -50
'gosub writeRegister_adxl
let reg = 0x20
let regval = -50
'gosub writeRegister_adxl
'Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register
let reg = POWER_CTL
let regval = 0x08
gosub writeRegister_adxl
'setup nrf

main:
	calibadc10 Nref			; Take the reference reading
	let regword = 52378 / Nref * 2		; Work out supply voltage
	let vlabel = "V"
	gosub sendOut
'for i = 0 to 5 step 2
'	let val16 = 0
'	let reg = i + 0x32
'	gosub readRegisterWord_adxl
'	let val16 = regword
'	gosub readRegister_adxl
'	let val16.0 = regval
'	inc reg
'	gosub readRegister_adxl
'	let val16.1 = regval
'	let vlabel = 0x30 + i
'	gosub sendOut
'	nap 1
'next i
let reg = 0
gosub readRegister_nrf
let regword = regval
let vlabel = "R"
gosub sendOut
high rf_ce
nap 1
low rf_ce
goto main

sendOut:
'	low rfselect
'	nap 1
'	high rfselect
'	nap 1
	sertxd(vlabel, ":",#regword,13,10)
	return

readRegister_adxl:
	high sclk
	low adxl_cs
	let var_out = reg | 0x80
	gosub shiftout_adxl
	gosub shiftin_adxl
	let regval = var_in
	high adxl_cs
	return

readRegisterWord_adxl:
	high sclk
	low adxl_cs
	let var_out = reg | 0xC0
	gosub shiftout_adxl
	gosub shiftin_adxl
	let regword.0 = var_in
	gosub shiftin_adxl
	let regword.1 = var_in
	high adxl_cs
	return

writeRegister_adxl:
	high sclk
	low adxl_cs
	let var_out = reg
	gosub shiftout_adxl
	let var_out = regval
	gosub shiftout_adxl
	high adxl_cs
	return



shiftin_adxl:
	let var_in = 0
	for counter = 1 to bits		; number of bits
		low sclk
		var_in = var_in * 2		; shift left as MSB first
		if serdata <> 0 then 
			var_in = var_in + 1		; set LSB if serdata = 1
		end if
		high sclk
		pause 1
	next counter
	return

shiftout_adxl:
	for counter = 1 to bits 	‘ number of bits
		low sclk
		mask = var_out & MSBValue 	‘ mask MSB
		if mask <> 0 then spi_high1
			low sdata
			goto spi_out1
spi_high1:
			high sdata
spi_out1:
		high sclk
		var_out = var_out * 2 		‘ shift variable left for MSB
	next counter
	return

readRegister_nrf:
'	low sclk
	low rf_cs
	let var_out = reg | 0x80
	gosub shiftout_nrf
	gosub shiftin_nrf
	let regval = var_in
	high rf_cs
	return

readRegisterWord_nrf:
'	low sclk
	low rf_cs
	let var_out = reg | 0xC0
	gosub shiftout_nrf
	gosub shiftin_nrf
	let regword.0 = var_in
	gosub shiftin_nrf
	let regword.1 = var_in
	high rf_cs
	return

writeRegister_nrf:
'	low sclk
	low rf_cs
	let var_out = reg
	gosub shiftout_nrf
	let var_out = regval
	gosub shiftout_nrf
	high rf_cs
	return

shiftin_nrf:
	let var_in = 0
	for counter = 1 to bits		; number of bits
'		high sclk
		var_in = var_in * 2		; shift left as MSB first
		if serdata <> 0 then 
			var_in = var_in + 1		; set LSB if serdata = 1
		end if
'		low sclk
	    pulsout sclk,1                   ' pulse clock to get next data bit
		pause 1
	next counter
	return

shiftout_nrf:
	for counter = 1 to bits 	‘ number of bits
'		high sclk
		mask = var_out & MSBValue 	‘ mask MSB
		if mask <> 0 then spi_high2
			low sdata
			goto spi_out2
spi_high2:
			high sdata
spi_out2:
'		low sclk
	    pulsout sclk,1                   ' pulse clock to get next data bit
		var_out = var_out * 2 		‘ shift variable left for MSB
	next counter
	return
