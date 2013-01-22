CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=HMC5883L.cpp hmc5883ld.cpp
OBJECTS=HMC5883L.o hmc5883ld.o
HEADERS=../ReServant/reservant.h ../I2CWire/I2CWire.h HMC5883L.h
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=hmc5883ld

.cpp.o: $(HEADERS)
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

HMC5883L.o: HMC5883L.h /usr/include/inttypes.h /usr/include/features.h
HMC5883L.o: /usr/include/stdint.h ../ReServant/reservant.h
HMC5883L.o: /usr/include/jansson.h /usr/include/stdio.h /usr/include/libio.h
HMC5883L.o: /usr/include/_G_config.h /usr/include/wchar.h
HMC5883L.o: /usr/include/stdlib.h /usr/include/alloca.h
HMC5883L.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
HMC5883L.o: /usr/include/hiredis/async.h ../I2CWire/I2CWire.h
HMC5883L.o: /usr/include/syslog.h /usr/include/math.h
hmc5883ld.o: /usr/include/stdio.h /usr/include/features.h
hmc5883ld.o: /usr/include/libio.h /usr/include/_G_config.h
hmc5883ld.o: /usr/include/wchar.h HMC5883L.h /usr/include/inttypes.h
hmc5883ld.o: /usr/include/stdint.h ../ReServant/reservant.h
hmc5883ld.o: /usr/include/jansson.h /usr/include/stdlib.h
hmc5883ld.o: /usr/include/alloca.h /usr/include/jansson_config.h
hmc5883ld.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
hmc5883ld.o: ../I2CWire/I2CWire.h
