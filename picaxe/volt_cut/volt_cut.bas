' power from 5v

' V_in - from lion divided by 5
symbol v_in = C.4
' mosfet pin (high - on / low - off)
symbol mosf_out = C.2

' with divider == (51+10)/10=6.1, 255 == (1.024 * 6.1)v so 1 of adc == 1.024 * 6.1 / 255 = 0.0245
' off voltage level 3.3v / 0.0245 = 135
' found by experiment
symbol v_off_level = 139
' on voltage level 3.6v / 0.0245 = 147
' found by experiment
symbol v_on_level = 149


' init

' initially set on
high mosf_out
' use fvrsetup
adcconfig %011
' mode: readadc reads 255 for 1.024v
fvrsetup FVR1024

' to have debug 4800 on serial comment setfreq
setfreq k31

main:

let b8 = v_in
gosub read_adc

if b1 <= v_off_level then
	low mosf_out
endif

if b1 >= v_on_level then
	high mosf_out
endif

' pause 1s = 1000 for 4MHz = 7 for 31k
' sleep, nap etc do not keep pins so do not work
pause 70

goto main

' b8 - pin
' w5 - accamulator
' b1 - temporary and return
read_adc:
	sertxd("READ START ")
	w5 = 0
	for b9 = 1 to 10
		readadc b8, b1
'		sertxd(#b8, ":", #w5, "+", #b1)
		let w5 = w5 + b1
'		sertxd("=", #w5, " ")
	next b9
	let b1 = w5 + 5 / 10
'	sertxd("ADC READ ", #b1,CR,LF)
	return
