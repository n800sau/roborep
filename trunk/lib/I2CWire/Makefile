CC=g++
CCOPTS=-L.
STRIP=-strip
OBJECTS=I2CWire.o
HEADERS=I2CWire.h
NAME=libi2cwire.a

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME): $(OBJECTS) $(HEADERS)
		$(AR) r $(NAME) $(OBJECTS)
