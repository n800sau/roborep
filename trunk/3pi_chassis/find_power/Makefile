DEVICE = atmega328p
MCU = atmega328p
AVRDUDE_DEVICE = m328p
DEVICE = atmega328p
MCU = atmega328p
AVRDUDE_DEVICE = m328p

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) -Os -I../../libpololu-avr -L../../libpololu-avr
CC=avr-gcc
OBJ2HEX=avr-objcopy 
LDFLAGS=-Wl,-gc-sections -lpololu_$(DEVICE) -Wl,-relax

PORT=usb
AVRDUDE=avrdude

TARGET=test
OBJECT_FILES=test.o



all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

slave.o: slave.c

utils.o: utils.c

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c usbasp -P $(PORT) -U flash:w:$(TARGET).hex

