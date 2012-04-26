package au.n800s.robo.model.3pi;

class HeadScannerThread extends Thread {
	private IOIO ioio;
	private Uart uart;
	private InputStream in;
	private OutputStream out;

	public HeadScannerThread(IOIO ioio, PwmOutput servo) throws ConnectionLostException  {
		this.ioio = ioio;
		uart = ioio.openUart(PinId.UART_USONIC_RX, PinId.UART_USONIC_TX, 9600, Uart.Parity.NONE, Uart.StopBits.ONE);
		in = uart.getInputStream();
		out = uart.getOutputStream();
	}

	protected byte[] requestData(byte cmd[], int answersize) throws IOException,InterruptedException 
	{
		byte receivedData[] = new byte[100];
		if(answersize > 100) {
			answersize = 100;
		}
		//DbMsg.i( "Sending command");
		out.write(cmd);
		sleep(10);
		receivedData[0] = 0;
		//DbMsg.i( "Reading reply...");
		in.read(receivedData,0 ,answersize);
		//DbMsg.i( "Reply received");
		receivedData[answersize] = 0;
		return receivedData; 
	}

	private String get_distance() throws IOException, InterruptedException {
		return new String(requestData(new byte[]{(byte)0x81}, 6), 0, 6);
	}

	@Override
	public void run() {
		try {
			Float(get_distance());
			Thread.sleep(1000);
			DbMsg.i("head end");
		} catch (Exception e) {
			DbMsg.e("Head Scanner stopped", e);
		}
	};
}
