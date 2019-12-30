CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=bmp085d.cpp BMP085.cpp
OBJECTS=bmp085d.o BMP085.o
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=bmp085d

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

bmp085d.o: BMP085.h ../ReServant/reservant.h /usr/include/jansson.h
bmp085d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
bmp085d.o: /usr/include/_G_config.h /usr/include/wchar.h
bmp085d.o: /usr/include/stdlib.h /usr/include/alloca.h
bmp085d.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
bmp085d.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
bmp085d.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
bmp085d.o: /usr/include/stdint.h /usr/include/netdb.h
bmp085d.o: /usr/include/netinet/in.h /usr/include/endian.h
bmp085d.o: /usr/include/rpc/netdb.h /usr/include/time.h
bmp085d.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h
BMP085.o: BMP085.h ../ReServant/reservant.h /usr/include/jansson.h
BMP085.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
BMP085.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
BMP085.o: /usr/include/alloca.h /usr/include/jansson_config.h
BMP085.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
BMP085.o: /usr/include/event2/http.h /usr/include/event2/util.h
BMP085.o: /usr/include/event2/event-config.h /usr/include/stdint.h
BMP085.o: /usr/include/netdb.h /usr/include/netinet/in.h
BMP085.o: /usr/include/endian.h /usr/include/rpc/netdb.h /usr/include/time.h
BMP085.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h /usr/include/unistd.h
BMP085.o: /usr/include/getopt.h /usr/include/math.h /usr/include/syslog.h
