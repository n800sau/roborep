symbol sclk = C.5			; clock (output pin)
symbol sdata = C.7			; data (output pin for shiftout)
symbol serdata_pin = C.6	; data (input pin for shiftin, note input7)
symbol serdata = pinC.6		; data (input pin for shiftin, note input7)
symbol cselect = C.4		; select adxl345
symbol rf_cs = C.3			; select nrf24l01+
symbol rf_ce = C.2			; ce nrf24l01+
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

input serdata_pin
output sdata
high cselect
high sclk

high rf_cs
high rf_ce
input rf_irq

input adxl_int1
input adxl_int2


'Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register
let reg = DATA_FORMAT
let regval = 0x08
'let regval = 0x05
gosub writeRegister
let reg = 0x1e
let regval = -50
'gosub writeRegister
let reg = 0x1f
let regval = -50
'gosub writeRegister
let reg = 0x20
let regval = -50
'gosub writeRegister
'Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register
let reg = POWER_CTL
let regval = 0x08
gosub writeRegister

main:
	calibadc10 Nref			; Take the reference reading
	let regword = 52378 / Nref * 2		; Work out supply voltage
	let vlabel = "V"
	gosub sendOut
for i = 0 to 5 step 2
	let val16 = 0
	let reg = i + 0x32
	gosub readRegisterWord
	let val16 = regword
'	gosub readRegister
'	let val16.0 = regval
'	inc reg
'	gosub readRegister
'	let val16.1 = regval
	let vlabel = 0x30 + i
	gosub sendOut
'	nap 1
next i
'nap 5
goto main

sendOut:
'	low rfselect
'	nap 1
'	high rfselect
'	nap 1
	sertxd(b12, ":",#regword,13,10)
	return

readRegister:
	low cselect
	let var_out = reg | 0x80
	gosub shiftout_MSBFirst
	gosub shiftin_MSB_Post
	let regval = var_in
	high cselect
	return

readRegisterWord:
	low cselect
	let var_out = reg | 0xC0
	gosub shiftout_MSBFirst
	gosub shiftin_MSB_Post
	let regword.0 = var_in
	gosub shiftin_MSB_Post
	let regword.1 = var_in
	high cselect
	return

writeRegister:
	low cselect
	let var_out = reg
	gosub shiftout_MSBFirst
	let var_out = regval
	gosub shiftout_MSBFirst
	high cselect
	return



shiftin_MSB_Post:
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

shiftout_MSBFirst:
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
