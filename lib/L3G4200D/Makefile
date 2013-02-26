CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=l3g4200dd.cpp L3G4200D.cpp Kalman.cpp
OBJECTS=l3g4200dd.o L3G4200D.o Kalman.o
HEADERS=../ReServant/reservant.h ../I2CWire/I2CWire.h L3G4200D.h
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=l3g4200dd

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

l3g4200dd.o: L3G4200D.h ../ReServant/reservant.h /usr/include/jansson.h
l3g4200dd.o: /usr/include/stdio.h /usr/include/features.h
l3g4200dd.o: /usr/include/libio.h /usr/include/_G_config.h
l3g4200dd.o: /usr/include/wchar.h /usr/include/stdlib.h /usr/include/alloca.h
l3g4200dd.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
l3g4200dd.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
l3g4200dd.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
l3g4200dd.o: /usr/include/stdint.h /usr/include/netdb.h
l3g4200dd.o: /usr/include/netinet/in.h /usr/include/endian.h
l3g4200dd.o: /usr/include/rpc/netdb.h /usr/include/time.h
l3g4200dd.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h
L3G4200D.o: L3G4200D.h ../ReServant/reservant.h /usr/include/jansson.h
L3G4200D.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
L3G4200D.o: /usr/include/_G_config.h /usr/include/wchar.h
L3G4200D.o: /usr/include/stdlib.h /usr/include/alloca.h
L3G4200D.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
L3G4200D.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
L3G4200D.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
L3G4200D.o: /usr/include/stdint.h /usr/include/netdb.h
L3G4200D.o: /usr/include/netinet/in.h /usr/include/endian.h
L3G4200D.o: /usr/include/rpc/netdb.h /usr/include/time.h
L3G4200D.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h /usr/include/math.h
L3G4200D.o: /usr/include/memory.h /usr/include/string.h /usr/include/syslog.h
L3G4200D.o: Kalman.h
Kalman.o: Kalman.h
