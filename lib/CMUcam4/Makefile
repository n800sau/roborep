CC=g++ -I../wiringPi/wiringPi
CCOPTS=-L.
STRIP=-strip
OBJECTS=CMUcam4.o CMUcom4.o
NAME=camP

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

libcmucam4.a: $(OBJECTS)
		$(AR) r libcmucam4.a $(OBJECTS)

clean:
	rm -f $(OBJECTS) $(NAME) libcmucam4.a

$(NAME):	$(NAME).cpp libcmucam4.a
	$(CC) -o $(NAME) $(NAME).cpp $(CCOPTS) libcmucam4.a ../wiringPi/wiringPi/libwiringPi.a
#	$(STRIP) -s $(NAME)
