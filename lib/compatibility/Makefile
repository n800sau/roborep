CC=g++
CCOPTS=-L.
STRIP=-strip
SRCS=compatibility.cpp
OBJECTS=compatibility.o
HEADERS=compatibility.h
NAME=libcompatibility.a

.cpp.o: $(HEADERS)
	$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME): $(OBJECTS) $(HEADERS)
	$(AR) r $(NAME) $(OBJECTS)
	ranlib $@

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

compatibility.o: compatibility.h /usr/include/time.h /usr/include/features.h
compatibility.o: /usr/include/xlocale.h
