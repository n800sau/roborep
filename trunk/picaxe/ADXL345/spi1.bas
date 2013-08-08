symbol sclk = C.1				; clock (output pin)
symbol sdata = C.2			; data (output pin for shiftout)
symbol serdata = pinC.3		; data (input pin for shiftin, note input7)
symbol cselect = C.4		; select output pin
symbol counter = b0			; variable used during loop
symbol mask = b1			; bit masking variable
symbol var_in = b2			; data variable used durig shiftin
symbol var_out = b3			; data variable used during shiftout
symbol reg = b4				; register
symbol regval = b5			; register value
symbol val16.0 = b10			; 16 bit value byte 1
symbol val16.1 = b11			; 16 bit value byte 2
symbol val16 = w5			; 16 bit value
symbol i = b20				; loop var
'symbol j = w11				; loop var
symbol bits = 8				; number of bits
symbol MSBvalue = 128			; MSBvalue =128 for 8 bits, 512 for 10 bits, 2048 for 12 bits)
symbol DATA_FORMAT = 0x31	;
symbol POWER_CTL = 0x2d		; Power Control Register

output sdata
high cselect
high sclk

'Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register
let reg = DATA_FORMAT
let regval = 0x05
gosub writeRegister
let reg = 0x1e
let regval = 100
gosub writeRegister
'Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register
let reg = POWER_CTL
let regval = 0x08
gosub writeRegister

'low cselect
'for counter = 1 to 10000
'	pulsin c.3,1,i
'	sertxd("Pulse ", #i,13,10)
'next counter
'high cselect


main:
for i = 0 to 5 step 2
	let val16 = 0
	let reg = i + 0x32
	gosub readRegister
	let val16.0 = var_in
	inc reg
	gosub readRegister
	let val16.1 = var_in
	sertxd("The register ", #i, " value is ",#val16,13,10)
'	nap 1
next i
'nap 5
sertxd(13,10)
goto main


readRegister:
	low cselect
	let var_out = reg | 0x80
	gosub shiftout_MSBFirst
	gosub shiftin_MSB_Post
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
'		count c.3, 1, j
'		sertxd("In bit = ", #j,13,10)
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
