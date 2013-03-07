CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant
STRIP=-strip
SRCS=rhttpd.cpp rhttp.cpp
OBJECTS=rhttpd.o rhttp.o
HEADERS=../ReServant/reservant.h rhttp.h
LIBS=../ReServant/libreservant.a
NAME=rhttpd

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

rhttpd.o: rhttp.h /usr/include/string.h /usr/include/features.h
rhttpd.o: /usr/include/xlocale.h ../ReServant/reservant.h
rhttpd.o: /usr/include/jansson.h /usr/include/stdio.h /usr/include/libio.h
rhttpd.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
rhttpd.o: /usr/include/alloca.h /usr/include/jansson_config.h
rhttpd.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
rhttpd.o: /usr/include/event2/http.h /usr/include/event2/util.h
rhttpd.o: /usr/include/event2/event-config.h /usr/include/stdint.h
rhttpd.o: /usr/include/netdb.h /usr/include/netinet/in.h
rhttpd.o: /usr/include/endian.h /usr/include/rpc/netdb.h /usr/include/time.h
rhttp.o: rhttp.h /usr/include/string.h /usr/include/features.h
rhttp.o: /usr/include/xlocale.h ../ReServant/reservant.h
rhttp.o: /usr/include/jansson.h /usr/include/stdio.h /usr/include/libio.h
rhttp.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
rhttp.o: /usr/include/alloca.h /usr/include/jansson_config.h
rhttp.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
rhttp.o: /usr/include/event2/http.h /usr/include/event2/util.h
rhttp.o: /usr/include/event2/event-config.h /usr/include/stdint.h
rhttp.o: /usr/include/netdb.h /usr/include/netinet/in.h /usr/include/endian.h
rhttp.o: /usr/include/rpc/netdb.h /usr/include/time.h /usr/include/syslog.h
rhttp.o: /usr/include/math.h /usr/include/evhttp.h /usr/include/event.h
rhttp.o: /usr/include/evutil.h /usr/include/event2/event_struct.h
rhttp.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
rhttp.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
rhttp.o: /usr/include/event2/buffer_compat.h
rhttp.o: /usr/include/event2/bufferevent.h
rhttp.o: /usr/include/event2/bufferevent_struct.h
rhttp.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
rhttp.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
rhttp.o: /usr/include/event2/http_compat.h /usr/include/unistd.h
rhttp.o: /usr/include/getopt.h /usr/include/libgen.h /usr/include/fcntl.h
