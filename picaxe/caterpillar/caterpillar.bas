#picaxe28X1								'set program editor mode to 28X1
#no_table
#no_data

'disablebod								'disable "brown out" detection

symbol fullspeed=100						'short time delay in mS
symbol halfspeed=250						'long time delay in mS

symbol turn=b3							'determines how sharp to turn and direction
symbol amount=b4							'variable used in when turning
symbol roll=b8							'roll sensor output
symbol getup=b11							'determine which method used for self righting
symbol IRmode=b12
symbol IRinput=3

poke 80,150								'store centre position of servo 0
poke 81,160								'store centre position of servo 1
poke 82,150								'store centre position of servo 2
poke 83,160								'store centre position of servo 3
poke 84,150								'store centre position of servo 4
poke 85,150								'store centre position of servo 5
poke 86,150								'store centre position of servo 6
poke 87,150								'store centre position of servo 7

symbol st0=7								'asign servo name to output pin (head servo)
symbol sm1=6								'asign servo name to output pin
symbol st2=5								'asign servo name to output pin
symbol sm3=4								'asign servo name to output pin
symbol st4=3								'asign servo name to output pin
symbol sm5=2								'asign servo name to output pin
symbol st6=1								'asign servo name to output pin
symbol sm7=0								'asign servo name to output pin (tail servo)

symbol rollsense=1						'assign analog pin to roll sensor

symbol Lant=0							'assign input pin to left antenna
symbol Rant=1							'assign input pin to right antenna
symbol Rear=2							'assign input pin to rear antenna

gosub straighten							'straighten body

let dirsc=%11110000						'set portc pins 4,5,6,7 to output for LEDs

for b2=1 to 20
	gosub lightchase						'generate light chase pattern to show program is running
	pause 50
next b2



main:{								
'goto noIR
	sertxd("Waiting IR",13,10)
	irin [200,noIR],IRinput,b2				'check for IR commands
	sertxd("The value of b2 is ",#b2,13,10)

	select case b2
	
	case 116							'if forward button is pressed, take a step forward
		gosub move
		goto main
	case 117							'if reverse button is pressed take a step back
		gosub back
		goto main
	case 51							'if left button is pressed, turn left
		turn=25
		gosub corner
		goto main
	case 52							'if right button is pressed, turn right
		turn=75
		gosub corner
		goto main
	case 101							'if center button is pressed then change IR mode between semi and full control
		IRmode=IRmode xor 1
		pause 2000
	case 0
		roll=127
		getup=0
		gosub recover2
		goto main
	case 1
		roll=175
		getup=0
		gosub recover2
		goto main
	end select
	
noIR:	
'	sertxd("noIR",13,10)

	readadc rollsense,roll					'check if caterpillar is upright
	sertxd("roll:",#roll,13,10)
	if roll<186 then						'caterpillar has fallen over
'		gosub straighten					'straighten body
		getup=0
'		gosub recover					'self righting routine
	end if
	
	if IRmode=1 then						'if IR mode is set to full control then goto main.
		gosub lightchase
		goto main
	end if

	inc b0							'boredom counter
	
	b1=pins and 7						'read head antenna inputs

'	sertxd("pins:",#b1,13,10)

'	pause 1000
'	goto main

	if b0<20 and b1=0 then			'if all clear and not bored then go forward
		gosub move
		goto main
	end if


	b0=0								'reset bordom counter
	b2=b1 and 1							'B2=inputs and tail
	if b2=0 then						'if tail is clear then backup
		gosub back
		gosub back
	end if
	peek 80,b10							'turn head to look left
	b10=b10+40
	servo st0,b10
	pause halfspeed						'wait for servo to move


'	readadc rangesense,b6					'check path to left					
	
	peek 80,b10							'turn head to look right
	b10=b10-40
	servo st0,b10
	pause halfspeed						'wait for servo to move
'	readadc rangesense,b7					'check path to right
	peek 80,b10					
	servo st0,b10						'turn head to look forward
	
'	if b6<b7 then						'turn in the direction that has more room
'		turn=75
'	else
'		turn=25
'	end if
	
	if b1 =2 then let turn=25 end if			'if antenna touch object then turn away
	if b1 =4 then let turn=75 end if
	if b1 =6 then						'if both antenna touch then back up more
		gosub back
		gosub back
		gosub corner					'extra turn if both antenna touch
	end if
	for b2=1 to 2
		gosub corner					'normal number of turns
	next b2
	goto main
	}

move:{								'This subroutine makes the caterpillar take a step forward
	
	gosub lightchase
	peek 81,b10
	b10=b10+40
	servo sm1,b10

	peek 83,b10
'	b10=b10-40
	b10=b10-60
	servo sm3,b10

	pause fullspeed
	gosub lightchase

	peek 87,b10
	b10=b10+30
	servo sm7,b10

	peek 85,b10
	b10=b10+30
	servo sm5,b10

	pause halfspeed
	gosub lightchase
	
	peek 85,b10
	b10=b10-40
	servo sm5,b10

	pause halfspeed
	gosub lightchase

	peek 83,b10
'	b10=b10+40
	b10=b10+60
	servo sm3,b10

	pause fullspeed
	gosub lightchase

	peek 87,b10
	b10=b10-40
	servo sm7,b10

	pause halfspeed
	gosub lightchase

	peek 85,b10
	servo sm5,b10

	peek 81,b10
	servo sm1,b10

	pause fullspeed
	gosub lightchase

	peek 83,b10
	servo sm3,b10

	peek 87,b10
	servo sm7,b10

	pause halfspeed

	return}

back:{								'This subroutine makes the caterpillar take a step backward
	
	peek 81,b10
	b10=b10+40
	servo sm1,b10
	peek 83,b10
	b10=b10+40
	servo sm3,b10
	peek 85,b10
	b10=b10-40
	servo sm5,b10
	peek 87,b10
	b10=b10-40
	servo sm7,b10
	pause halfspeed
	
	peek 87,b10
	b10=b10+40
	servo sm7,b10
	peek 83,b10
	b10=b10-40
	servo sm3,b10
	pause halfspeed
	gosub lightchase
	
	peek 87,b10
	servo sm7,b10
	peek 85,b10
	servo sm5,b10
	pause fullspeed
	gosub lightchase
	peek 83,b10
	servo sm3,b10
	peek 81,b10
	servo sm1,b10
	pause halfspeed
	
	return}

corner:{								'This subroutine makes the caterpillar turn left or right
									'the variable "turn" determins how sharp to turn and direction
	peek 81,b10
	b10=b10+40
	servo sm1,b10
	peek 83,b10
	b10=b10-40
	servo sm3,b10
	peek 87,b10
	b10=b10+40
	servo sm7,b10
	pause fullspeed
	gosub lightchase
	
	amount=100+turn
	servo st4,amount
	
	amount=200-turn
	servo st6,amount
	
	amount=200-turn
	servo st2,amount
	pause fullspeed
	gosub lightchase
	
	peek 81,b10
	servo sm1,b10
	peek 83,b10
	servo sm3,b10
	peek 87,b10
	servo sm7,b10
	pause fullspeed
	gosub lightchase
	
	peek 85,b10
	b10=b10+40
	servo sm5,b10
	peek 87,b10
	b10=b10+40
	servo sm7,b10
	pause fullspeed
	gosub lightchase
	
	amount=100+turn
	servo st6,amount
	pause fullspeed
	gosub lightchase
	
	peek 83,b10
	b10=b10+30
	servo sm3,b10
	pause fullspeed
	gosub lightchase
	
	peek 82,b10
	servo st2,b10
	peek 84,b10
	servo st4,b10
	peek 86,b10
	servo st6,b10
	pause fullspeed
	gosub lightchase
	
	peek 83,b10
	servo sm3,b10
	peek 85,b10
	servo sm5,b10
	peek 87,b10
	servo sm7,b10
	pause fullspeed
	gosub lightchase
	
	return}
	
recover:{								'This subroutine tells the caterpillar how to get up if it falls over
	pause 300
	readadc rollsense,roll					'check for false alarm           
	if roll>185 then						'if ok then return
		return
	end if
recover2:	
	if getup=0 then						'first method of self righting
		peek 81,b10						'curl up
		b10=b10-50
		servo sm1,b10
		peek 83,b10
		b10=b10+50
		servo sm3,b10
		peek 85,b10
		b10=b10-50
		servo sm5,b10
		peek 87,b10
		b10=b10+50
		servo sm7,b10
	else
		peek 81,b10
		b10=b10+50
		servo sm1,b10
		peek 83,b10
		b10=b10-50
		servo sm3,b10
		peek 85,b10
		b10=b10+50
		servo sm5,b10
		peek 87,b10
		b10=b10-50
		servo sm7,b10
	end if
	pause 300
	gosub lightchase
	
	'readadc rollsense,roll					'check body position
	'if roll>185 then						'if ok then return
	'	return
	'end if
	if getup=1 then						'second method of self righting 
		roll=roll+100
	end if
		
	if roll>149 then						'twist left
		peek 86,b10
		b10=b10-50
		servo st6,b10
		peek 82,b10
		b10=b10-50
		servo st2,b10
		peek 84,b10
		b10=b10+50
		servo st4,b10	
	else								'twist right
		peek 86,b10
		b10=b10+50
		servo st6,b10
		peek 82,b10
		b10=b10+50
		servo st2,b10
		peek 84,b10
		b10=b10-50
		servo st4,b10
	end if
	
	for b0=1 to 4
		peek 81,b10
		servo sm1,b10
		peek 87,b10
		servo sm7,b10
		pause 90
		peek 81,b10
		b10=b10-50
		servo sm1,b10
		peek 87,b10
		b10=b10+50
		servo sm7,b10
		pause 100
		gosub lightchase
	next b0
		
	for b0=1 to 2
		pause 300
		gosub lightchase
	next b0
			
	peek 80,b10
	servo st0,b10							'slowly uncurl
	pause 200
	peek 81,b10
	servo sm1,b10
	peek 87,b10
	servo sm7,b10
	pause 200
	gosub lightchase
	peek 85,b10
	servo sm5,b10
	peek 83,b10
	servo sm3,b10
	pause 200
	peek 82,b10
	servo st2,b10
	peek 84,b10
	servo st4,b10
	pause 100
	gosub lightchase
	peek 86,b10
	servo st6,b10
	pause 300
	gosub lightchase
		
	readadc rollsense,roll
	if roll>185 then						'if ok then return
		return
	end if
	let getup=getup xor 1					'toggle method of getting up if still not on feet
	goto recover
	}
	
straighten:{
	
	peek 80,b10
	servo st0,b10						'straighten body
	peek 81,b10
	servo sm1,b10
	peek 82,b10
	servo st2,b10
	peek 83,b10
	servo sm3,b10
	peek 84,b10
	servo st4,b10
	peek 85,b10
	servo sm5,b10
	peek 86,b10
	servo st6,b10
	peek 87,b10
	servo sm7,b10
	pause 200
	gosub lightchase
	return}	
		
lightchase:{
	b9=b9*2
	if b9<16 then
		b9=16
	end if
	let pinsc = b9
	return}
