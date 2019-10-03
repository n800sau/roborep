using namespace std;
 
#include <iostream>
#include <serialCom.h>
 
/* Flag indicating datas are ready to be read */
static bool flag = false;
 
/* SIGIO handler */
void
my_handler(int status)
{
  /* Data ready to be read */
  flag = true;
}
 
/* MAIN */
int
main(int argc, char **argv)
{
  /* Initialize the serial communication */
  int fdesc = serialConfiguration(my_handler, "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0", B57600,
      NO_PARITY_CHECK);
 
  /* Data buffer */
  char buf;
  /* Reading result */
  int res = 0;
 
  /* Initialize data flag */
  flag = false;
 
  while (buf != '\t')
    {
      /* If there is a data to be read */
      if (flag == true)
        {
          /* Read one byte */
          do
            {
              res = read(fdesc, &buf, 1);
              cout << buf;
            }
          while (res > 0);
 
          flag = false;
        }
    }
 
  return 0;
}
