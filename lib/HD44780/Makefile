CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant
ifeq ($(wildcard .PIMAKE),) 
	CCOPTS+=-I../BeagleBone_IO_lib -DBB
else 
	CCOPTS+=-DPI
endif
STRIP=-strip
SRCS=hd44780d.cpp HD44780.cpp
OBJECTS=hd44780d.o HD44780.o
LIBS=../ReServant/libreservant.a ../BeagleBone_IO_lib/BeagleBone_IO_lib.a
NAME=hd44780d




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

hd44780d.o: HD44780.h ../ReServant/reservant.h /usr/include/jansson.h
hd44780d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
hd44780d.o: /usr/include/_G_config.h /usr/include/wchar.h
hd44780d.o: /usr/include/stdlib.h /usr/include/alloca.h
hd44780d.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
hd44780d.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
hd44780d.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
hd44780d.o: /usr/include/stdint.h /usr/include/netdb.h
hd44780d.o: /usr/include/netinet/in.h /usr/include/endian.h
hd44780d.o: /usr/include/rpc/netdb.h /usr/include/time.h
hd44780d.o: /usr/include/xlocale.h ../BeagleBone_IO_lib/BeagleBone_gpio.h
hd44780d.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/string.h
hd44780d.o: /usr/include/assert.h ../BeagleBone_IO_lib/BeagleBone_hd44780.h
HD44780.o: HD44780.h ../ReServant/reservant.h /usr/include/jansson.h
HD44780.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
HD44780.o: /usr/include/_G_config.h /usr/include/wchar.h
HD44780.o: /usr/include/stdlib.h /usr/include/alloca.h
HD44780.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
HD44780.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
HD44780.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
HD44780.o: /usr/include/stdint.h /usr/include/netdb.h
HD44780.o: /usr/include/netinet/in.h /usr/include/endian.h
HD44780.o: /usr/include/rpc/netdb.h /usr/include/time.h
HD44780.o: /usr/include/xlocale.h ../BeagleBone_IO_lib/BeagleBone_gpio.h
HD44780.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/string.h
HD44780.o: /usr/include/assert.h ../BeagleBone_IO_lib/BeagleBone_hd44780.h
HD44780.o: /usr/include/syslog.h /usr/include/math.h /usr/include/evhttp.h
HD44780.o: /usr/include/event.h /usr/include/evutil.h
HD44780.o: /usr/include/event2/event_struct.h
HD44780.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
HD44780.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
HD44780.o: /usr/include/event2/buffer_compat.h
HD44780.o: /usr/include/event2/bufferevent.h
HD44780.o: /usr/include/event2/bufferevent_struct.h
HD44780.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
HD44780.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
HD44780.o: /usr/include/event2/http_compat.h /usr/include/libgen.h
HD44780.o: /usr/include/fcntl.h /usr/include/ifaddrs.h /usr/include/errno.h
HD44780.o: /usr/include/arpa/inet.h
