CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -g
STRIP=-strip
SRCS=userverd.cpp userver.cpp
OBJECTS=userverd.o userver.o
HEADERS=../ReServant/reservant.h userver.h
LIBS=../ReServant/libreservant.a
NAME=userverd

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

userverd.o: userver.h /usr/include/string.h /usr/include/features.h
userverd.o: /usr/include/bits/predefs.h /usr/include/sys/cdefs.h
userverd.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
userverd.o: /usr/include/gnu/stubs-64.h /usr/include/xlocale.h
userverd.o: ../ReServant/reservant.h /usr/include/jansson.h
userverd.o: /usr/include/stdio.h /usr/include/bits/types.h
userverd.o: /usr/include/bits/typesizes.h /usr/include/libio.h
userverd.o: /usr/include/_G_config.h /usr/include/wchar.h
userverd.o: /usr/include/bits/wchar.h /usr/include/bits/stdio_lim.h
userverd.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
userverd.o: /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h
userverd.o: /usr/include/endian.h /usr/include/bits/endian.h
userverd.o: /usr/include/bits/byteswap.h /usr/include/sys/types.h
userverd.o: /usr/include/time.h /usr/include/sys/select.h
userverd.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
userverd.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
userverd.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
userverd.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
userverd.o: /usr/include/sys/time.h /usr/include/hiredis/async.h
userverd.o: /usr/include/event2/http.h /usr/include/event2/util.h
userverd.o: /usr/include/event2/event-config.h /usr/include/stdint.h
userverd.o: /usr/include/netdb.h /usr/include/netinet/in.h
userverd.o: /usr/include/sys/socket.h /usr/include/sys/uio.h
userverd.o: /usr/include/bits/uio.h /usr/include/bits/socket.h
userverd.o: /usr/include/bits/sockaddr.h /usr/include/asm/socket.h
userverd.o: /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h
userverd.o: /usr/include/asm-generic/sockios.h /usr/include/bits/in.h
userverd.o: /usr/include/rpc/netdb.h /usr/include/bits/netdb.h
userverd.o: /usr/include/event2/bufferevent.h
userver.o: userver.h /usr/include/string.h /usr/include/features.h
userver.o: /usr/include/bits/predefs.h /usr/include/sys/cdefs.h
userver.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
userver.o: /usr/include/gnu/stubs-64.h /usr/include/xlocale.h
userver.o: ../ReServant/reservant.h /usr/include/jansson.h
userver.o: /usr/include/stdio.h /usr/include/bits/types.h
userver.o: /usr/include/bits/typesizes.h /usr/include/libio.h
userver.o: /usr/include/_G_config.h /usr/include/wchar.h
userver.o: /usr/include/bits/wchar.h /usr/include/bits/stdio_lim.h
userver.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
userver.o: /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h
userver.o: /usr/include/endian.h /usr/include/bits/endian.h
userver.o: /usr/include/bits/byteswap.h /usr/include/sys/types.h
userver.o: /usr/include/time.h /usr/include/sys/select.h
userver.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
userver.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
userver.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
userver.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
userver.o: /usr/include/sys/time.h /usr/include/hiredis/async.h
userver.o: /usr/include/event2/http.h /usr/include/event2/util.h
userver.o: /usr/include/event2/event-config.h /usr/include/stdint.h
userver.o: /usr/include/netdb.h /usr/include/netinet/in.h
userver.o: /usr/include/sys/socket.h /usr/include/sys/uio.h
userver.o: /usr/include/bits/uio.h /usr/include/bits/socket.h
userver.o: /usr/include/bits/sockaddr.h /usr/include/asm/socket.h
userver.o: /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h
userver.o: /usr/include/asm-generic/sockios.h /usr/include/bits/in.h
userver.o: /usr/include/rpc/netdb.h /usr/include/bits/netdb.h
userver.o: /usr/include/event2/bufferevent.h /usr/include/syslog.h
userver.o: /usr/include/sys/syslog.h /usr/include/bits/syslog-path.h
userver.o: /usr/include/math.h /usr/include/bits/huge_val.h
userver.o: /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h
userver.o: /usr/include/bits/inf.h /usr/include/bits/nan.h
userver.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
userver.o: /usr/include/evhttp.h /usr/include/event.h /usr/include/evutil.h
userver.o: /usr/include/event2/event_struct.h
userver.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
userver.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
userver.o: /usr/include/event2/buffer_compat.h
userver.o: /usr/include/event2/bufferevent_struct.h
userver.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
userver.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
userver.o: /usr/include/event2/http_compat.h /usr/include/unistd.h
userver.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
userver.o: /usr/include/getopt.h /usr/include/ctype.h /usr/include/libgen.h
userver.o: /usr/include/fcntl.h /usr/include/bits/fcntl.h
userver.o: /usr/include/bits/stat.h /usr/include/sys/stat.h
userver.o: /usr/include/arpa/inet.h
