#include <SparkFunMPU9250-DMP.h>

#define SerialPort Serial

MPU9250_DMP imu;

#define LED_PIN 13

void setup() 
{
	SerialPort.begin(115200);
	pinMode(LED_PIN, OUTPUT);

	// Call imu.begin() to verify communication and initialize
	while (imu.begin() != INV_SUCCESS)
	{
		SerialPort.println("Unable to communicate with MPU-9250");
		SerialPort.println("Check connections, and try again.");
		SerialPort.println();
		delay(5000);
	}
	// Enable 6-axis quat
	// Use gyro calibration
	// Set DMP FIFO rate to default 100 Hz (200 (MAX_DMP_SAMPLE_RATE) maximum)
	// DMP_FEATURE_LP_QUAT can also be used. It uses the 
	// accelerometer in low-power mode to estimate quat's.
	// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
	if(imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO, 100) != INV_SUCCESS) {
		Serial.println("IMU DMP start fail");
	} else {
		Serial.println("IMU Ready");
	}
}

void loop() 
{
	// Check for new data in the FIFO
	if ( imu.fifoAvailable() )
	{
		// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
		if ( imu.dmpUpdateFifo() == INV_SUCCESS)
		{
			digitalWrite(LED_PIN, HIGH);
			// computeEulerAngles can be used -- after updating the
			// quaternion values -- to estimate roll, pitch, and yaw
			imu.computeEulerAngles();
			printIMUData();
		} else {
			digitalWrite(LED_PIN, LOW);
		}
	} else {
		digitalWrite(LED_PIN, LOW);
	}
	delay(50);
}

void printIMUData(void)
{
	// After calling dmpUpdateFifo() the ax, gx, mx, etc. values
	// are all updated.
	// Quaternion values are, by default, stored in Q30 long
	// format. calcQuat turns them into a float between -1 and 1
	float q0 = imu.calcQuat(imu.qx);
	float q1 = imu.calcQuat(imu.qy);
	float q2 = imu.calcQuat(imu.qz);
	float q3 = imu.calcQuat(imu.qw);

	SerialPort.println("Q: " + String(q0, 4) + " " + String(q1, 4) + " " + String(q2, 4) + " " + String(q3, 4));
	SerialPort.println("RPY: " + String(imu.roll) + " " + String(imu.pitch) + " " + String(imu.yaw));
	SerialPort.println("AV: " + String(imu.calcGyro(imu.gx)) + " " + String(imu.calcGyro(imu.gx)) + " " + String(imu.calcGyro(imu.gz)));
	SerialPort.println("LA: " + String(imu.calcAccel(imu.ax)) + " " + String(imu.calcAccel(imu.ax)) + " " + String(imu.calcAccel(imu.az)));
	SerialPort.println("Time: " + String(imu.time) + " ms");
	SerialPort.println();
}

