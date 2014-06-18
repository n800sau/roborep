CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -g
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

grhttpd.o: rhttp.h /usr/include/string.h /usr/include/features.h
grhttpd.o: /usr/include/xlocale.h ../ReServant/reservant.h
grhttpd.o: /usr/include/jansson.h /usr/include/stdio.h /usr/include/libio.h
grhttpd.o: /usr/include/_G_config.h /usr/include/wchar.h
grhttpd.o: /usr/include/stdlib.h /usr/include/alloca.h
grhttpd.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
grhttpd.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
grhttpd.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
grhttpd.o: /usr/include/stdint.h /usr/include/netdb.h
grhttpd.o: /usr/include/netinet/in.h /usr/include/endian.h
grhttpd.o: /usr/include/rpc/netdb.h /usr/include/time.h
grhttpd.o: /usr/include/event2/bufferevent.h
grhttp.o: rhttp.h /usr/include/string.h /usr/include/features.h
grhttp.o: /usr/include/xlocale.h ../ReServant/reservant.h
grhttp.o: /usr/include/jansson.h /usr/include/stdio.h /usr/include/libio.h
grhttp.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
grhttp.o: /usr/include/alloca.h /usr/include/jansson_config.h
grhttp.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
grhttp.o: /usr/include/event2/http.h /usr/include/event2/util.h
grhttp.o: /usr/include/event2/event-config.h /usr/include/stdint.h
grhttp.o: /usr/include/netdb.h /usr/include/netinet/in.h
grhttp.o: /usr/include/endian.h /usr/include/rpc/netdb.h /usr/include/time.h
grhttp.o: /usr/include/event2/bufferevent.h /usr/include/syslog.h
grhttp.o: /usr/include/math.h /usr/include/evhttp.h /usr/include/event.h
grhttp.o: /usr/include/evutil.h /usr/include/event2/event_struct.h
grhttp.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
grhttp.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
grhttp.o: /usr/include/event2/buffer_compat.h
grhttp.o: /usr/include/event2/bufferevent_struct.h
grhttp.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
grhttp.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
grhttp.o: /usr/include/event2/http_compat.h /usr/include/unistd.h
grhttp.o: /usr/include/getopt.h /usr/include/libgen.h /usr/include/fcntl.h
