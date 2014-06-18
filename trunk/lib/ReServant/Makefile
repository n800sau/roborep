CC=g++ -I/usr/include/hiredis
CCOPTS=-L. -g
STRIP=-strip
SRCS=reservant.cpp hiredis_ext.cpp
OBJECTS=reservant.o hiredis_ext.o
NAME=libreservant.a

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME): $(OBJECTS) $(HEADERS)
	$(AR) r $(NAME) $(OBJECTS)
	ranlib $@

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

greservant.o: reservant.h /usr/include/jansson.h /usr/include/stdio.h
greservant.o: /usr/include/features.h /usr/include/libio.h
greservant.o: /usr/include/_G_config.h /usr/include/wchar.h
greservant.o: /usr/include/stdlib.h /usr/include/alloca.h
greservant.o: /usr/include/jansson_config.h /usr/include/event2/http.h
greservant.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
greservant.o: /usr/include/stdint.h /usr/include/netdb.h
greservant.o: /usr/include/netinet/in.h /usr/include/endian.h
greservant.o: /usr/include/rpc/netdb.h /usr/include/time.h
greservant.o: /usr/include/xlocale.h /usr/include/event2/bufferevent.h
greservant.o: hiredis_ext.h /usr/include/linux/limits.h /usr/include/math.h
greservant.o: /usr/include/syslog.h /usr/include/signal.h
greservant.o: /usr/include/malloc.h /usr/include/string.h
greservant.o: /usr/include/unistd.h /usr/include/getopt.h
greservant.o: /usr/include/libgen.h /usr/include/fcntl.h /usr/include/errno.h
greservant.o: /usr/include/arpa/inet.h /usr/include/event2/listener.h
greservant.o: /usr/include/event2/event.h /usr/include/event2/buffer.h
ghiredis_ext.o: /usr/include/stdlib.h /usr/include/features.h
ghiredis_ext.o: /usr/include/alloca.h /usr/include/string.h
ghiredis_ext.o: /usr/include/xlocale.h /usr/include/syslog.h hiredis_ext.h
ghiredis_ext.o: /usr/include/stdio.h /usr/include/libio.h
ghiredis_ext.o: /usr/include/_G_config.h /usr/include/wchar.h
