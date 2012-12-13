CC=g++ -I../wiringPi/wiringPi -I/usr/include/hiredis
CCOPTS=-L.
STRIP=-strip
OBJECTS=CMUcam4.o CMUcom4.o
HEADERS=CMUcam4.h CMUcom4.h
NAME=camTr

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

libcmucam4.a: $(OBJECTS) $(HEADERS)
		$(AR) r libcmucam4.a $(OBJECTS)

clean:
	rm -f $(OBJECTS) $(NAME) libcmucam4.a

$(NAME):	$(NAME).cpp libcmucam4.a
	$(CC) -o $(NAME) $(NAME).cpp $(CCOPTS) libcmucam4.a ../wiringPi/wiringPi/libwiringPi.a -lopencv_core -lopencv_highgui -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)
