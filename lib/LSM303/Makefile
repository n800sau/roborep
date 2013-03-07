CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=lsm303d.cpp LSM303.cpp
OBJECTS=lsm303d.o LSM303.o
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=lsm303d

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

lsm303d.o: LSM303.h ../ReServant/reservant.h /usr/include/jansson.h
lsm303d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
lsm303d.o: /usr/include/_G_config.h /usr/include/wchar.h
lsm303d.o: /usr/include/stdlib.h /usr/include/alloca.h
lsm303d.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
lsm303d.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
lsm303d.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
lsm303d.o: /usr/include/stdint.h /usr/include/netdb.h
lsm303d.o: /usr/include/netinet/in.h /usr/include/endian.h
lsm303d.o: /usr/include/rpc/netdb.h /usr/include/time.h
lsm303d.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h
LSM303.o: LSM303.h ../ReServant/reservant.h /usr/include/jansson.h
LSM303.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
LSM303.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
LSM303.o: /usr/include/alloca.h /usr/include/jansson_config.h
LSM303.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
LSM303.o: /usr/include/event2/http.h /usr/include/event2/util.h
LSM303.o: /usr/include/event2/event-config.h /usr/include/stdint.h
LSM303.o: /usr/include/netdb.h /usr/include/netinet/in.h
LSM303.o: /usr/include/endian.h /usr/include/rpc/netdb.h /usr/include/time.h
LSM303.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h /usr/include/unistd.h
LSM303.o: /usr/include/getopt.h /usr/include/math.h
