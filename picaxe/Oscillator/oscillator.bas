symbol irpin = c.1 ' Define output pin for pulses
symbol devid = 36

main:

irout irpin, devid, 16

nap 5

goto main
