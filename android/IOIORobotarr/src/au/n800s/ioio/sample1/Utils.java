public class Utile {

	public int MAXVAL =  0x1FFF;
	public int MINVAL = -0x1FFF;

	static public byte bytes2short(byte lb, byte hb)
	{
		short rs = (((short)hb & 0x3F)<<7) + lb;
		if(hb & 0x40) {
			rs = -rs;
		}
		return rs;
	}

	static public byte[] short2bytes(short val)
	{
		byte[] bytes = new byte[2];
		short ainv = (val<0) ? -val : val;
		bytes[0]= ainv & 0x7F;
		bytes[1] = (ainv >> 7) & 0x3F;
		if(val < 0) {
			bytes[1] |= 0x40;
		}
		return bytes;
	}

}


