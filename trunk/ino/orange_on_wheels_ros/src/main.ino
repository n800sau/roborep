#include <ros.h>

#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"

const int echoPin = 12; // Echo Pin
const int trigPin = 11; // Trigger Pin


ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mf_msg;

ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::Publisher pub_imu( "/imu", &imu_msg);
ros::Publisher pub_mf( "/mf", &mf_msg);

const char range_frameid[] = "/ultrasound";
const char imu_frameid[] = "/imu";

float getRange_Ultrasound()
{
	long duration = -1; // Duration used to calculate distance
//	noInterrupts();
	// The following trigPin/echoPin cycle is used to determine the
	// distance of the nearest object by bouncing soundwaves off of it.
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);

	digitalWrite(trigPin, LOW);
//	interrupts();
	duration = pulseIn(echoPin, HIGH);

	//Calculate the distance (in m) based on the speed of sound.
	return duration / 58.2 / 100;
}


void setup()
{
	setup_compass();
	setup_accel();
	setup_gyro();

	nh.initNode();
	nh.advertise(pub_range);
	nh.advertise(pub_imu);
	nh.advertise(pub_mf);

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.header.frame_id = range_frameid;
	range_msg.field_of_view = 0.1; // fake
	range_msg.min_range = 0.0;
	range_msg.max_range = 2.0;

	imu_msg.header.frame_id = imu_frameid;

}


unsigned long msg_time = 0;

int act = 0;

void loop()
{
	//publish the adc value every 50 milliseconds
	//since it takes that long for the sensor to stablize
	if ( millis() >= msg_time ){
		switch(act) {
			case 0:
				range_msg.range = getRange_Ultrasound();
				range_msg.header.stamp = nh.now();
				pub_range.publish(&range_msg);
				break;
			case 1:
				process_compass();
				break;
			case 2:
				mf_msg.header.stamp = nh.now();
				mf_msg.magnetic_field.x = compass_x;
				mf_msg.magnetic_field.y = compass_y;
				mf_msg.magnetic_field.z = compass_z;
				pub_mf.publish(&mf_msg);
				break;
			case 3:
				process_accel();
				break;
			case 4:
				process_gyro();
				break;
			case 5:
				imu_msg.header.stamp = nh.now();
				imu_msg.angular_velocity.x = gyro.g.x;
				imu_msg.angular_velocity.y = gyro.g.y;
				imu_msg.angular_velocity.z = gyro.g.z;
				imu_msg.linear_acceleration.x = acc_x_avg();
				imu_msg.linear_acceleration.y = acc_y_avg();
				imu_msg.linear_acceleration.z = acc_z_avg();
				pub_imu.publish(&imu_msg);
				break;
		}
		act = (act+1) % 6;
		msg_time = millis() + 50;
	}
	nh.spinOnce();
}
