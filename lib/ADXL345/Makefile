CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=adxl345d.cpp ADXL345.cpp
OBJECTS=adxl345d.o ADXL345.o
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

adxl345d.o: ADXL345.h ../ReServant/reservant.h /usr/include/jansson.h
adxl345d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
adxl345d.o: /usr/include/_G_config.h /usr/include/wchar.h
adxl345d.o: /usr/include/stdlib.h /usr/include/alloca.h
adxl345d.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
adxl345d.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
adxl345d.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
adxl345d.o: /usr/include/stdint.h /usr/include/netdb.h
adxl345d.o: /usr/include/netinet/in.h /usr/include/endian.h
adxl345d.o: /usr/include/rpc/netdb.h /usr/include/time.h
adxl345d.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h
ADXL345.o: ADXL345.h ../ReServant/reservant.h /usr/include/jansson.h
ADXL345.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
ADXL345.o: /usr/include/_G_config.h /usr/include/wchar.h
ADXL345.o: /usr/include/stdlib.h /usr/include/alloca.h
ADXL345.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
ADXL345.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
ADXL345.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
ADXL345.o: /usr/include/stdint.h /usr/include/netdb.h
ADXL345.o: /usr/include/netinet/in.h /usr/include/endian.h
ADXL345.o: /usr/include/rpc/netdb.h /usr/include/time.h
ADXL345.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h /usr/include/syslog.h
ADXL345.o: /usr/include/math.h /usr/include/evhttp.h /usr/include/event.h
ADXL345.o: /usr/include/evutil.h /usr/include/event2/event_struct.h
ADXL345.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
ADXL345.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
ADXL345.o: /usr/include/event2/buffer_compat.h
ADXL345.o: /usr/include/event2/bufferevent.h
ADXL345.o: /usr/include/event2/bufferevent_struct.h
ADXL345.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
ADXL345.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
ADXL345.o: /usr/include/event2/http_compat.h /usr/include/string.h
ADXL345.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/libgen.h
ADXL345.o: /usr/include/fcntl.h
