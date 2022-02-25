' power from 5v

' V_in - from lion divided by 5
symbol v_in = C.4
symbol V_lion = w5
' mosfet pin (high - on / low - off)
symbol mosf_out = C.2

' with divider == 5, 255 == (1.024 * 5)v so 1 of adc == 1.024 * 5 / 255 = 0.02
' off voltage level 4v / 0.02 = 200
symbol v_off_level = 200
' on voltage level 4.6v / 0.02 = 230
symbol v_on_level = 230


' init

' initially set on
high mosf_out
' use fvrsetup
adcconfig %011
' mode: readadc reads 255 for 1.024v
fvrsetup FVR1024

main:

let b8 = v_in
gosub read_adc

if V_lion <= v_off_level then
	low mosf_out
endif

if V_lion >= v_on_level then
	high mosf_out
endif

disablebod
' 2.3s * 10 = 23s
sleep 10
enablebod

goto main

' b8 - pin
' w5 - accamulator
' b1 - temporary
read_adc:
	sertxd("READ START ")
	w5 = 0
	for b9 = 1 to 10
		readadc b8, b1
		sertxd(#b8, ":", #w5, "+", #b1)
		let w5 = w5 + b1
		sertxd("=", #w5, " ")
	next b9
	let b1 = w5 + 5 / 10
	sertxd("STOP ", #w5,CR,LF)
	return
