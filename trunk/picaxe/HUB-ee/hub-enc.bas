; Creative Robotics Ltd 2013
; PICAXE Shield Base example for HUB-ee Prototyping Shield
; Distributed unter the Creative Commons Attribution-ShareAlike 3.0 Unported License (CC BY-SA 3.0)
; Spins each wheel (in turn) forward at full power, then reverse at half power

;M1 L 16,14,11,6,8,4   - c.6,b.4,b.7,c.3,c.2,b.2   qb qa stby pwm in2 in1 pw grn
;M2 R 15,13,9,5,7,3	   - b.3,b.5,c.1,c.5,c.4,c.7   qb qa stby pwm in2 in1 pw grn

; new
;M1 L 16,14,11,6,8,4   - c.6,c.1,b.7,c.3,b.4,b.2   qb qa stby pwm in2 in1	pw grn
;M2 R 15,13,9,5,7,3	   - c.4,c.2,c.7,c.5,b.3,b.5   qb qa stby pwm in2 in1	pw grn
;c.1 - c.5 can be used for setint

symbol M1PWMPin = C.3
symbol M1In1Pin = B.2
symbol M1In2Pin = B.4
symbol M1qA = pinC.1
symbol M1qb = pinC.6

; Power can be from 0 - 1023
symbol M1Power = w0
; Direction: 0 = direction A, 1 or more = direction B
symbol M1Direction = b4
; q count
symbol M1qcnt = w3

; old M1qA
symbol oldM1qA = b10

symbol M2PWMPin = C.5
symbol M2In1Pin = B.5
symbol M2In2Pin = B.3
symbol M2qA = pinC.2
symbol M2qb = pinC.4

; Power can be from 0 - 1023
symbol M2Power = w1
; Direction: 0 = direction A, 1 or more = direction B
symbol M2Direction = b5
; q count
symbol M2qcnt = w4

; old M2qA
symbol oldM2qA = b11

; set picaxe type
#picaxe 20m2

setup:
PWMOUT PWMDIV16, M1PWMPin, 255, 0
PWMOUT PWMDIV16, M2PWMPin, 255, 0
gosub setup_int
goto main

main:
	pause 1000
	gosub print_data

	goto main



	; Spin Motor1 in direction A at full speed
	; Set the variables
	M1Power = 1023
	M1Direction = 0
	; Call subroutine to update the direction outputs and PWM
	gosub setM1

	pause 1000
	
	; Stop Motor1
	; Set the variables (power to zero)
	M1Power = 0
	; Call subroutine to update the direction outputs and PWM
	gosub setM1

	; Spin Motor2 in direction A at full speed
	; Set the variables
	M2Power = 1023
	M2Direction = 0
	; Call subroutine to update the direction outputs and PWM
	gosub setM2

	pause 1000
	; Stop Motor2
	; Set the variables (power to zero)
	let M2Power = 0
	; Call subroutine to update the direction outputs and PWM
	gosub setM2
	
	; Spin Motor1 in direction B at half speed
	; Set the variables
	M1Power = 511
	M1Direction = 1
	; Call subroutine to update the direction outputs and PWM
	gosub setM1

	pause 1000

	; Stop Motor1
	; Set the variables (power to zero)
	M1Power = 0
	; Call subroutine to update the direction outputs and PWM
	gosub setM1

	; Spin Motor2 in direction B at half speed
	; Set the variables
	M2Power = 511
	M2Direction = 1
	; Call subroutine to update the direction outputs and PWM
	gosub setM2

	pause 1000
	; Stop Motor2
	; Set the variables (power to zero)
	let M2Power = 0
	; Call subroutine to update the direction outputs and PWM
	gosub setM2

	goto main

setM1:
	PWMDUTY M1PWMPin,M1Power
	if M1Direction = 1 then
	low M1In1Pin
	high M1In2Pin
	else
	high M1In1Pin
	low M1In2Pin
	endif
	return

setM2:
	PWMDUTY M2PWMPin,M2Power
	if M2Direction = 1 then
	low M2In1Pin
	high M2In2Pin
	else
	high M2In1Pin
	low M2In2Pin
	endif
	return

print_data:
	sertxd (#M1qcnt, "\t", #M2qcnt, 13, 10)
	return

interrupt:
	gosub M1_QEIupdate
	gosub M2_QEIupdate
	gosub setup_int
	return

setup_int:
	if oldM2qA=0 and oldM1qA=0 then
		setint or %00000110,%00000110
	elseif oldM2qA=0 and oldM1qA<>0 then
		setint or %00000100,%00000110
	elseif oldM2qA<>0 and oldM1qA=0 then
		setint or %00000010,%00000110
	elseif oldM2qA<>0 and oldM1qA<>0 then
		setint or %00000000,%00000110
	end if
	return

M1_QEIupdate:
	if M1qA <> oldM1qA then
		if M1qA = 1 then
			if M1qB = 1 then
				dec M1qcnt
			else
				inc M1qcnt
			end if
		else
			if M1qB = 1 then
				inc M1qcnt
			else
				dec M1qcnt
			end if
		end if
		oldM1qA = M1qA
	end if
	do while oldM1qA = M1qA
		pause 100
	loop
	return

M2_QEIupdate:
	if M2qA <> oldM2qA then
		if M2qA = 1 then
			if M2qB = 1 then
				dec M2qcnt
			else
				inc M2qcnt
			end if
		else
			if M2qB = 1 then
				inc M2qcnt
			else
				dec M2qcnt
			end if
		end if
		oldM2qA = M2qA
	end if
	do while oldM2qA = M2qA
		pause 100
	loop
	return

