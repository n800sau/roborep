CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=adxl345d.cpp ADXL345.cpp
OBJECTS=adxl345d.o ADXL345.o
HEADERS=../ReServant/reservant.h ../I2CWire/I2CWire.h ADXL345.h
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=adxl345d

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

adxl345d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
adxl345d.o: /usr/include/_G_config.h /usr/include/wchar.h ADXL345.h
adxl345d.o: ../ReServant/reservant.h ../I2CWire/I2CWire.h
adxl345d.o: /usr/include/stdint.h
ADXL345.o: ADXL345.h ../ReServant/reservant.h ../I2CWire/I2CWire.h
ADXL345.o: /usr/include/stdint.h /usr/include/features.h
ADXL345.o: /usr/include/syslog.h /usr/include/math.h
