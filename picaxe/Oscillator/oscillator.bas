symbol irpin = c.1 ' Define output pin for pulses
symbol devid = 36
symbol ledpin = c.2

low ledpin

main:

irout irpin, devid, 16

nap 5
high ledpin
nap 1
low ledpin


goto main
