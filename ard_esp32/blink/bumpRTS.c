#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


/*
TIOCM_LE        DSR (data set ready/line enable)
TIOCM_DTR       DTR (data terminal ready)
TIOCM_RTS       RTS (request to send)
TIOCM_ST        Secondary TXD (transmit)
TIOCM_SR        Secondary RXD (receive)
TIOCM_CTS       CTS (clear to send)
TIOCM_CAR       DCD (data carrier detect)
TIOCM_CD         see TIOCM_CAR
TIOCM_RNG       RNG (ring)
TIOCM_RI         see TIOCM_RNG
TIOCM_DSR       DSR (data set ready)
*/

static struct termios oldterminfo;


void closeserial(int fd)
{
//	if(tcsetattr(fd, TCSANOW, &oldterminfo) == -1) {
//		perror("closeserial(): tcsetattr()");
//	}
	if (close(fd) < 0)
		perror("closeserial()");
}


int openserial(char *devicename)
{
	int fd;
	struct termios attr;

//	if ((fd = open(devicename, O_RDWR)) == -1) {
	if ((fd = open(devicename, O_RDWR | O_NOCTTY)) == -1) {
		perror("openserial(): open()");
		return 0;
	}
//	if (tcgetattr(fd, &oldterminfo) == -1) {
//		perror("openserial(): tcgetattr()");
//		return 0;
//	}
//	attr = oldterminfo;
//	attr.c_cflag |= CRTSCTS | CLOCAL;
//	attr.c_oflag = 0;
//	if (tcflush(fd, TCIOFLUSH) == -1) {
//		perror("openserial(): tcflush()");
//		return 0;
//	}
//	if (tcsetattr(fd, TCSANOW, &attr) == -1) {
//		perror("initserial(): tcsetattr()");
//		return 0;
//	}
	return fd;
}


int setBit(int fd, int bit, int level) {
	if (ioctl(fd, level ? TIOCMBIS : TIOCMBIC, &bit) == -1) {
		perror("setBit(): TIOCMSET");
		return 0;
	}
	return 1;
}

int setRTS(int fd, int level)
{
	return setBit(fd, TIOCM_RTS, level);
}

int setDTR(int fd, int level)
{
	return setBit(fd, TIOCM_DTR, level);
}


int main(int argc, char **argv)
{
	int fd;
	char *serialdev = argv[1];

	fd = openserial(serialdev);
	if (!fd) {
		fprintf(stderr, "Error while initializing %s.\n", serialdev);
		return 1;
	}

//	for(int i=0; i<2; i++) {
		setDTR(fd, 1);
		setRTS(fd, 0);
//		setBit(fd, TIOCM_CTS, 0);
//		setBit(fd, TIOCM_RI, 0);
		sleep(1);
		setDTR(fd, 1);
		sleep(1);
		setRTS(fd, 0);
//		setBit(fd, TIOCM_CTS, 0);
//		setBit(fd, TIOCM_RI, 1);
		sleep(1);
//	}

	closeserial(fd);
	return 0;
}
