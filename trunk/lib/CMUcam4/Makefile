CC=g++
CCOPTS=-L.
STRIP=-strip
OBJECTS=CMUcam4.o CMUcom4.o
NAME=camTest

.cpp.o:
		$(CC) $(CCOPTS) -c $<

libcmucam4.a: $(OBJECTS)
		$(AR) r libcmucam4.a $(OBJECTS)

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME) libcmucam4.a

$(NAME):	$(NAME).c
	$(CC) -o $(NAME) $(NAME).c $(CCOPTS) libcmucam4.a
	$(STRIP) -s $(NAME)
