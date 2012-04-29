package au.n800s.robo.model.track;

import au.n800s.robo.common.DbMsg;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;

class LEDThread extends Thread {
	private static final long LED_BLINK_SPEED = 500;
	private DigitalOutput led;
	private IOIO ioio;

	public LEDThread(IOIO ioio) {
		this.ioio = ioio;
	}

	@Override
	public void run() {
		try {
			boolean ledState = true;
			led = ioio.openDigitalOutput(IOIO.LED_PIN, ledState);
			while (true) {
				ledState = !ledState;
				led.write(ledState);
				Thread.sleep(LED_BLINK_SPEED);
			}
		} catch (Exception e) {
			DbMsg.e("LED thread is stopped", e);
		}
	}
}
