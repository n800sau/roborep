CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=mpu6050d.cpp MPU6050.cpp
OBJECTS=mpu6050d.o MPU6050.o
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=mpu6050d

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

