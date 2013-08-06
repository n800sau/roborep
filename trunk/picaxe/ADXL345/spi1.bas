symbol sclk = C.1				; clock (output pin)
symbol sdata = C.2			; data (output pin for shiftout)
symbol serdata = pinC.3			; data (input pin for shiftin, note input7)
symbol cselect = C.4		; select output pin
symbol counter = b0			; variable used during loop
symbol mask = b1			; bit masking variable
symbol var_in = b2			; data variable used durig shiftin
symbol var_out = b3			; data variable used during shiftout
symbol reg = b4				; register
symbol regval = b5			; register value
symbol i = b12				; loop var
symbol bits = 8				; number of bits
symbol MSBvalue = 128			; MSBvalue =128 for 8 bits, 512 for 10 bits, 2048 for 12 bits)


output sclk, sdata, cselect

high cselect

let reg = 0x31
gosub readRegister
sertxd("The register ", #reg, " value is ",#var_in,13,10)
let regval = var_in & 0xBF
gosub writeRegister


main:
for i = 0 to 5
	let reg = i + 0x32
	gosub readRegister
	sertxd("The register ", #reg, " value is ",#var_in,13,10)
next i
nap 5
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
	  var_in = var_in * 2		; shift left as MSB first
	  pulsout sclk,1		; pulse clock to get next data bit
	  if serdata <> 0 then 
	    var_in = var_in + 1		; set LSB if serdata = 1
	  end if
	next counter
	return

‘ ***** Shiftout MSB first *****
shiftout_MSBFirst:
	for counter = 1 to bits 	‘ number of bits
	mask = var_out & MSBValue 	‘ mask MSB
	high sdata 			‘ data high
	if mask = 0 then skipMSB
	low sdata 			‘ data low
skipMSB: pulsout sclk,1 		‘ pulse clock for 10us
	var_out = var_out * 2 		‘ shift variable left for MSB
	next counter
	return
