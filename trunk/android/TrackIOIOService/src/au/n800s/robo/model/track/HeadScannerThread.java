package au.n800s.robo.model.track;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import au.n800s.robo.common.DbMsg;

import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;

class HeadScannerThread extends Thread {
	private IOIO ioio;
	private PwmOutput pwmOutput;
	private Uart uart;
	private InputStream in;
	private OutputStream out;

	public HeadScannerThread(IOIO ioio, PwmOutput servo) throws ConnectionLostException  {
		this.ioio = ioio;
		pwmOutput = servo;
		pwmOutput.setPulseWidth(1500);
		DbMsg.i("centered");
		//			uart = ioio.openUart(PinId.UART_USONIC_RX, PinId.UART_USONIC_TX, 9600, Uart.Parity.NONE, Uart.StopBits.ONE);
		//	        in = uart.getInputStream();
		//	        out = uart.getOutputStream();
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
			DbMsg.i("head start");
			//turn to one side
			for(int i = 1500; i < 2500; i += 10) {
				pwmOutput.setPulseWidth(i);
				DbMsg.i("i="+i);
				Thread.sleep(100);
			}
			Thread.sleep(1000);
			//turn to other side
			for(int i = 2500; i > 500; i -= 10) {
				pwmOutput.setPulseWidth(i);
				DbMsg.i("i="+i);
				Thread.sleep(100);
			}
			Thread.sleep(1000);
			//return to the middle position
			for(int i = 500; i < 1500; i += 10) {
				pwmOutput.setPulseWidth(i);
				DbMsg.i("i="+i);
				Thread.sleep(100);
			}
			Thread.sleep(1000);
			DbMsg.i("head end");
		} catch (Exception e) {
			DbMsg.e("Head Scanner stopped", e);
		}
	};
}
