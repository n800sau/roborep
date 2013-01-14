CC=g++ -I/usr/include/hiredis
CCOPTS=-L.
STRIP=-strip
OBJECTS=reservant.o
HEADERS=reservant.h
NAME=libreservant.a

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME): $(OBJECTS) $(HEADERS)
		$(AR) r $(NAME) $(OBJECTS)
