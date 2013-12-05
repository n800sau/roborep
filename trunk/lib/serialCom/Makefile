CC=g++
CCOPTS=-L. -g
STRIP=-strip
SRCS=serialCom.c
OBJECTS=serialCom.o
NAME=libserialCom.a

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

serialCom.o: /usr/include/fcntl.h /usr/include/features.h
serialCom.o: /usr/include/bits/predefs.h /usr/include/sys/cdefs.h
serialCom.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
serialCom.o: /usr/include/gnu/stubs-64.h /usr/include/bits/fcntl.h
serialCom.o: /usr/include/sys/types.h /usr/include/bits/types.h
serialCom.o: /usr/include/bits/typesizes.h /usr/include/time.h
serialCom.o: /usr/include/endian.h /usr/include/bits/endian.h
serialCom.o: /usr/include/bits/byteswap.h /usr/include/sys/select.h
serialCom.o: /usr/include/bits/select.h /usr/include/bits/sigset.h
serialCom.o: /usr/include/bits/time.h /usr/include/sys/sysmacros.h
serialCom.o: /usr/include/bits/pthreadtypes.h /usr/include/bits/stat.h
serialCom.o: /usr/include/sys/signal.h /usr/include/signal.h
serialCom.o: /usr/include/bits/signum.h /usr/include/bits/siginfo.h
serialCom.o: /usr/include/bits/sigaction.h /usr/include/bits/sigcontext.h
serialCom.o: /usr/include/bits/sigstack.h /usr/include/sys/ucontext.h
serialCom.o: /usr/include/bits/sigthread.h /usr/include/unistd.h
serialCom.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
serialCom.o: /usr/include/getopt.h /usr/include/ctype.h
serialCom.o: /usr/include/xlocale.h /usr/include/stdio.h /usr/include/libio.h
serialCom.o: /usr/include/_G_config.h /usr/include/wchar.h
serialCom.o: /usr/include/bits/wchar.h /usr/include/bits/stdio_lim.h
serialCom.o: /usr/include/bits/sys_errlist.h /usr/include/termios.h
serialCom.o: /usr/include/bits/termios.h /usr/include/sys/ttydefaults.h
