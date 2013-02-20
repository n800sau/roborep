CC=g++
CCOPTS=-L. -I../ReServant -I../wiringPi -I../CMUcam4 -I/usr/include/hiredis
STRIP=-strip
SRCS=camServant.cpp
OBJECTS=camServant.o
LIBS=../CMUcam4/libcmucam4.a ../wiringPi/libwiringPi.a ../ReServant/libreservant.a
NAME=camServant

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(CCOPTS) $(OBJECTS) $(LIBS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

camServant.o: ../ReServant/reservant.h /usr/include/jansson.h /usr/include/stdio.h
camServant.o: /usr/include/features.h /usr/include/libio.h
camServant.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
camServant.o: /usr/include/alloca.h /usr/include/jansson_config.h
camServant.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
camServant.o: /usr/include/syslog.h /usr/include/time.h /usr/include/xlocale.h
camServant.o: /usr/include/unistd.h /usr/include/getopt.h ../CMUcam4/CMUcam4.h
camServant.o: ../CMUcam4/CMUcom4.h /usr/include/stdint.h /usr/include/setjmp.h
camServant.o: /usr/include/string.h
