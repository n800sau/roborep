symbol onpin = c.1
symbol irpin = c.2

Symbol Zero = 240
Symbol One = 120

pwmout irpin, 17, 36; Sets a frequency of 56 kHz on Pin c.2

Main:

	Low onpin ;allows current to pass
	Pause One
	High onpin ; stops current
	Pause Zero

goto Main
