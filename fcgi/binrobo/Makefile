CC=g++ -I../../lib/wiringPi/wiringPi -I../../lib/CMUcam4 -I/usr/include/fastcgi
CCOPTS=-L. -lfcgi
STRIP=-strip
OBJECTS=server.o
HEADERS=
NAME=server

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(NAME).cpp ../../lib/CMUcam4/libcmucam4.a
	$(CC) -o $(NAME) $(NAME).cpp $(CCOPTS) ../../lib/CMUcam4/libcmucam4.a ../../lib/wiringPi/wiringPi/libwiringPi.a -lopencv_core
	$(STRIP) -s $(NAME)
