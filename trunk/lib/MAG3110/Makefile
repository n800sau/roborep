CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=mag3110d.cpp MAG3110.cpp
OBJECTS=mag3110d.o MAG3110.o
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=mag3110d

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

mag3110d.o: MAG3110.h ../ReServant/reservant.h /usr/include/jansson.h
mag3110d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
mag3110d.o: /usr/include/_G_config.h /usr/include/wchar.h
mag3110d.o: /usr/include/stdlib.h /usr/include/alloca.h
mag3110d.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
mag3110d.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
mag3110d.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
mag3110d.o: /usr/include/stdint.h /usr/include/netdb.h
mag3110d.o: /usr/include/netinet/in.h /usr/include/endian.h
mag3110d.o: /usr/include/rpc/netdb.h /usr/include/time.h
mag3110d.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h
MAG3110.o: MAG3110.h ../ReServant/reservant.h /usr/include/jansson.h
MAG3110.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
MAG3110.o: /usr/include/_G_config.h /usr/include/wchar.h
MAG3110.o: /usr/include/stdlib.h /usr/include/alloca.h
MAG3110.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
MAG3110.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
MAG3110.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
MAG3110.o: /usr/include/stdint.h /usr/include/netdb.h
MAG3110.o: /usr/include/netinet/in.h /usr/include/endian.h
MAG3110.o: /usr/include/rpc/netdb.h /usr/include/time.h
MAG3110.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h /usr/include/syslog.h
MAG3110.o: /usr/include/math.h /usr/include/evhttp.h /usr/include/event.h
MAG3110.o: /usr/include/evutil.h /usr/include/event2/event_struct.h
MAG3110.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
MAG3110.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
MAG3110.o: /usr/include/event2/buffer_compat.h
MAG3110.o: /usr/include/event2/bufferevent.h
MAG3110.o: /usr/include/event2/bufferevent_struct.h
MAG3110.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
MAG3110.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
MAG3110.o: /usr/include/event2/http_compat.h /usr/include/string.h
MAG3110.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/libgen.h
MAG3110.o: /usr/include/fcntl.h
