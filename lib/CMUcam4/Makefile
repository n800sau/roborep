CC=g++
CCOPTS=-L. -I../wiringPi
STRIP=-strip
SRCS=CMUcam4.cpp CMUcom4.cpp
OBJECTS=CMUcam4.o CMUcom4.o
HEADERS=CMUcam4.h CMUcom4.h
NAME=libcmucam4.a

.cpp.o:
	$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

lint:
	lint $(CCOPTS) $(SRCS)

$(NAME): $(OBJECTS) $(HEADERS)
		$(AR) r $(NAME) $(OBJECTS)
		ranlib $@

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

CMUcam4.o: CMUcam4.h CMUcom4.h /usr/include/stdlib.h /usr/include/features.h
CMUcam4.o: /usr/include/alloca.h /usr/include/stdint.h /usr/include/setjmp.h
CMUcam4.o: /usr/include/stdio.h /usr/include/libio.h /usr/include/_G_config.h
CMUcam4.o: /usr/include/wchar.h /usr/include/string.h /usr/include/xlocale.h
CMUcom4.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
CMUcom4.o: /usr/include/_G_config.h /usr/include/wchar.h
CMUcom4.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/time.h
CMUcom4.o: /usr/include/xlocale.h /usr/include/string.h CMUcom4.h
CMUcom4.o: /usr/include/stdlib.h /usr/include/alloca.h /usr/include/stdint.h
CMUcom4.o: ../wiringPi/wiringSerial.h
