CC=g++
CCOPTS=-L. -I../wiringPi -I../CMUcam4 -I/usr/include/hiredis
STRIP=-strip
OBJECTS=camTr.o
HEADERS=../CMUcam4/CMUcam4.h ../CMUcam4/CMUcom4.h
LIBS=../CMUcam4/libcmucam4.a ../wiringPi/libwiringPi.a
NAME=camTr

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(HEADERS) $(OBJECTS)
	$(CC) -o $(NAME) $(CCOPTS) $(OBJECTS) $(LIBS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)
