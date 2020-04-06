package au.n800s.ioio.sample1;

public class Utils {

	public static int MAXVAL =  0x1FFF;
	public static int MINVAL = -0x1FFF;

	static public short bytes2short(byte lb, byte hb)
	{
		int rs = (((short)hb & 0x3F)<<7) + lb;
		if((((short)hb) & 0x40) != 0) {
			rs = -rs;
		}
		return (short) rs;
	}

	static public byte[] short2bytes(short val)
	{
		byte[] bytes = new byte[2];
		int ainv = (val<0) ? -val : val;
		bytes[0]= (byte) (ainv & 0x7F);
		bytes[1] = (byte) ((ainv >> 7) & 0x3F);
		if(val < 0) {
			bytes[1] |= 0x40;
		}
		return bytes;
	}

}


