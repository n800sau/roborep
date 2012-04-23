package au.n800s.robo.model.track;

	class LEDThread extends Thread {
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

