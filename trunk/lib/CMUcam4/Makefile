CC=g++
CCOPTS=-L. -I../wiringPi
STRIP=-strip
OBJECTS=CMUcam4.o CMUcom4.o
HEADERS=CMUcam4.h CMUcom4.h
NAME=libcmucam4.a

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME): $(OBJECTS) $(HEADERS)
		$(AR) r $(NAME) $(OBJECTS)
