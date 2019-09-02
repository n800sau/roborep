//#include <stdio.h>
//#include <math.h>

#define MAXVAL 0x1FFF
#define MINVAL -0x1FFF

short bytes2short(char lb, char hb)
{
	short rs = (((short)hb & 0x3F)<<7) + lb;
	if(hb & 0x40) {
		rs = -rs;
	}
	return rs;
}

void short2bytes(short val, char *bytes)
{
	short ainv = (val<0) ? -val : val;
	bytes[0]= ainv & 0x7F;
	bytes[1] = (ainv >> 7) & 0x3F;
	if(val < 0) {
		bytes[1] |= 0x40;
	}
}

/*int main()
{
	char bytes[2];
	short2bytes(-200, bytes);
	printf("%x, %x\n", bytes[0], bytes[1]);
	short val = bytes2short(bytes[0], bytes[1]);
	printf("%x, %d\n", val, val);
	return 0;
}*/
