// to get distabnce from 8m2 HC-SR04 serial

#include <ros/ros.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <oculus2wd/distance_sensor.h>

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		fprintf (stderr, "error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	// 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;		// ignore break signal
	tty.c_lflag = 0;		// no signaling chars, no echo,
					// no canonical processing
	tty.c_oflag = 0;		// no remapping, no delays
	tty.c_cc[VMIN]	= 0;		// read doesn't block
	tty.c_cc[VTIME] = 5;		// 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
					// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		fprintf (stderr, "error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		fprintf (stderr, "error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]	= should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;		// 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		fprintf (stderr, "error %d setting term attributes", errno);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "distance_ranger");
	ros::NodeHandle nh;
	std::string tty_dev;
	nh.param("tty", tty_dev, std::string("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"));
	printf("tty=%s\n", tty_dev.c_str());
	//const char *portname = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0";
	//const char *portname = "/dev/serial/by-id/usb-Revolution_AXE027_PICAXE_USB-if00-port0";
	int fd = open (tty_dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf (stderr, "error %d opening %s: %s", errno, tty_dev.c_str(), strerror (errno));
		return 1;
	}
	set_interface_attribs (fd, B4800, 0);	// set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);	// set no blocking
	ros::Publisher pub = nh.advertise<oculus2wd::distance_sensor> ("/oculus2wd/head_distance", 1);
	char buf[100];
	int pos = 0, n;
	while(ros::ok()) {
		n = read(fd, buf+pos, sizeof(char));
		if(n > 0) {
			if(buf[pos] == '\n') {
//			printf("buf=%s\n", buf);
				buf[pos] = 0;
				oculus2wd::distance_sensor msg;
				//Range:17
				char *cptr = strchr(buf, ':');
				if(cptr) {
					msg.distance = atoi(cptr + sizeof(char));
					if(msg.distance > 0) {
						pub.publish(msg);
					}
				}
				// a line has been read
				pos = 0;
			} else {
				pos += n;
			}
		}
		ros::spinOnce();
	}
}
