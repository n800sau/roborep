CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -g
STRIP=-strip
SRCS=tcpserverd.cpp tcpserver.cpp
OBJECTS=tcpserverd.o tcpserver.o
HEADERS=../ReServant/reservant.h tcpserver.h
LIBS=../ReServant/libreservant.a
NAME=tcpserverd

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

tcpserverd.o: tcpserver.h /usr/include/string.h /usr/include/features.h
tcpserverd.o: /usr/include/bits/predefs.h /usr/include/sys/cdefs.h
tcpserverd.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
tcpserverd.o: /usr/include/gnu/stubs-64.h /usr/include/xlocale.h
tcpserverd.o: ../ReServant/reservant.h /usr/include/jansson.h
tcpserverd.o: /usr/include/stdio.h /usr/include/bits/types.h
tcpserverd.o: /usr/include/bits/typesizes.h /usr/include/libio.h
tcpserverd.o: /usr/include/_G_config.h /usr/include/wchar.h
tcpserverd.o: /usr/include/bits/wchar.h /usr/include/bits/stdio_lim.h
tcpserverd.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
tcpserverd.o: /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h
tcpserverd.o: /usr/include/endian.h /usr/include/bits/endian.h
tcpserverd.o: /usr/include/bits/byteswap.h /usr/include/sys/types.h
tcpserverd.o: /usr/include/time.h /usr/include/sys/select.h
tcpserverd.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
tcpserverd.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
tcpserverd.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
tcpserverd.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
tcpserverd.o: /usr/include/sys/time.h /usr/include/hiredis/async.h
tcpserverd.o: /usr/include/event2/http.h /usr/include/event2/util.h
tcpserverd.o: /usr/include/event2/event-config.h /usr/include/stdint.h
tcpserverd.o: /usr/include/netdb.h /usr/include/netinet/in.h
tcpserverd.o: /usr/include/sys/socket.h /usr/include/sys/uio.h
tcpserverd.o: /usr/include/bits/uio.h /usr/include/bits/socket.h
tcpserverd.o: /usr/include/bits/sockaddr.h /usr/include/asm/socket.h
tcpserverd.o: /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h
tcpserverd.o: /usr/include/asm-generic/sockios.h /usr/include/bits/in.h
tcpserverd.o: /usr/include/rpc/netdb.h /usr/include/bits/netdb.h
tcpserverd.o: /usr/include/event2/bufferevent.h
tcpserver.o: tcpserver.h /usr/include/string.h /usr/include/features.h
tcpserver.o: /usr/include/bits/predefs.h /usr/include/sys/cdefs.h
tcpserver.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
tcpserver.o: /usr/include/gnu/stubs-64.h /usr/include/xlocale.h
tcpserver.o: ../ReServant/reservant.h /usr/include/jansson.h
tcpserver.o: /usr/include/stdio.h /usr/include/bits/types.h
tcpserver.o: /usr/include/bits/typesizes.h /usr/include/libio.h
tcpserver.o: /usr/include/_G_config.h /usr/include/wchar.h
tcpserver.o: /usr/include/bits/wchar.h /usr/include/bits/stdio_lim.h
tcpserver.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
tcpserver.o: /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h
tcpserver.o: /usr/include/endian.h /usr/include/bits/endian.h
tcpserver.o: /usr/include/bits/byteswap.h /usr/include/sys/types.h
tcpserver.o: /usr/include/time.h /usr/include/sys/select.h
tcpserver.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
tcpserver.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
tcpserver.o: /usr/include/bits/pthreadtypes.h /usr/include/alloca.h
tcpserver.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
tcpserver.o: /usr/include/sys/time.h /usr/include/hiredis/async.h
tcpserver.o: /usr/include/event2/http.h /usr/include/event2/util.h
tcpserver.o: /usr/include/event2/event-config.h /usr/include/stdint.h
tcpserver.o: /usr/include/netdb.h /usr/include/netinet/in.h
tcpserver.o: /usr/include/sys/socket.h /usr/include/sys/uio.h
tcpserver.o: /usr/include/bits/uio.h /usr/include/bits/socket.h
tcpserver.o: /usr/include/bits/sockaddr.h /usr/include/asm/socket.h
tcpserver.o: /usr/include/asm-generic/socket.h /usr/include/asm/sockios.h
tcpserver.o: /usr/include/asm-generic/sockios.h /usr/include/bits/in.h
tcpserver.o: /usr/include/rpc/netdb.h /usr/include/bits/netdb.h
tcpserver.o: /usr/include/event2/bufferevent.h /usr/include/syslog.h
tcpserver.o: /usr/include/sys/syslog.h /usr/include/bits/syslog-path.h
tcpserver.o: /usr/include/math.h /usr/include/bits/huge_val.h
tcpserver.o: /usr/include/bits/huge_valf.h /usr/include/bits/huge_vall.h
tcpserver.o: /usr/include/bits/inf.h /usr/include/bits/nan.h
tcpserver.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
tcpserver.o: /usr/include/evhttp.h /usr/include/event.h /usr/include/evutil.h
tcpserver.o: /usr/include/event2/event_struct.h
tcpserver.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
tcpserver.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
tcpserver.o: /usr/include/event2/buffer_compat.h
tcpserver.o: /usr/include/event2/bufferevent_struct.h
tcpserver.o: /usr/include/event2/bufferevent_compat.h
tcpserver.o: /usr/include/event2/tag.h /usr/include/event2/tag_compat.h
tcpserver.o: /usr/include/event2/http_struct.h
tcpserver.o: /usr/include/event2/http_compat.h /usr/include/unistd.h
tcpserver.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
tcpserver.o: /usr/include/getopt.h /usr/include/ctype.h /usr/include/libgen.h
tcpserver.o: /usr/include/fcntl.h /usr/include/bits/fcntl.h
tcpserver.o: /usr/include/bits/stat.h /usr/include/sys/stat.h
tcpserver.o: /usr/include/arpa/inet.h
