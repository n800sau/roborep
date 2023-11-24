#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include "data_struct.h"

static serial::Serial ser;

S_RECV_BUF recv_buf;

static uint16_t calc_crc(uint8_t *buf, unsigned sz)
{
	uint16_t rs = 0xA0A0;
	for(unsigned i=0; i<sz; i++) {
		rs += (((uint16_t)buf[i]) << 8) + (buf[i] - 1);
	}
	return rs;
}

static bool read_data(uint8_t *buf, unsigned sz)
{
	bool rs = false;
	uint16_t r_crc, crc;
	if(ser.read(buf, sz) == sz) {
		if(ser.read((uint8_t *)&r_crc, sizeof(r_crc)) == sizeof(r_crc)) {
//		ROS_DEBUG("read crc %4.4x", r_crc);
			crc = calc_crc(buf, sz);
//		ROS_DEBUG("calc crc %4.4x", crc);
			if(crc == r_crc) {
				rs = true;
			}
		}
	}
	return rs;
}

static STYPE receive()
{
	STYPE rs = ST_NONE;
	uint8_t b;
//	ser.flush();
	if(ser.read((uint8_t *)&b, 1)) {
		if(b == MARKER) {
			if(ser.read((uint8_t *)&b, 1)) {
				switch(b) {
					case ST_TSQ:
						rs = (STYPE)b;
						break;
					case ST_ODOM:
						if(read_data((uint8_t *)&recv_buf, sizeof(S_ODOM))) {
							rs = (STYPE)b;
						}
						break;
				}
			}
		}
	}
	return rs;
}

void send(STYPE tp, void *buf, unsigned sz)
{
	ser.write(&MARKER, 1);
	ser.write((uint8_t*)&tp, 1);
	if(buf) {
		ser.write((uint8_t*)buf, sz);
		uint16_t crc = calc_crc((uint8_t*)buf, sz);
		ser.write((uint8_t*)&crc, sizeof(crc));
	}
}

bool vel_cmd = false;
S_VEL vel_buf;

void velCallback(const geometry_msgs::Twist::ConstPtr& vel) {
	vel_buf.lx = vel->linear.x;
	vel_buf.th = vel->angular.z;
	vel_cmd = true;
}

const char base_link_id[] = "base_link";
const char odom_id[] = "odom";

int main(int argc, char** argv)
{
	std::string port;
	ros::init(argc, argv, "pipe_node");

	ros::NodeHandle private_node_handle("~");
	private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");

	ros::NodeHandle nh("pipe");

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 0, velCallback);

	ros::Rate r(200); // 200 hz

	ros::Time current_time;


	while(ros::ok())
	{
		try
		{
			if (ser.isOpen())
			{
				STYPE t = receive();
				if(t != ST_NONE) {
//					ROS_DEBUG("got %.2x", (int)t);
					switch(t) {
						case ST_TSQ:
							recv_buf.ts.val = (uint32_t)time(NULL);
	//						ROS_DEBUG("send %d", recv_buf.ts.val);
							send(ST_TSR, (uint8_t*)&recv_buf.ts, sizeof(recv_buf.ts));
							if(vel_cmd) {
								send(ST_VEL, &vel_buf, sizeof(vel_buf));
								ROS_DEBUG("sent vel %g,%g", vel_buf.lx, vel_buf.th);
								vel_cmd = false;
							}
							break;
						case ST_VEL:
							ROS_DEBUG("recv vel %g,%g", recv_buf.vel.lx, recv_buf.vel.th);
							break;
						case ST_ODOM:
							ROS_DEBUG("recv odom");
							// recv_buf.odom
							current_time = ros::Time::now();
							//since all odometry is 6DOF we'll need a quaternion created from yaw
							geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(recv_buf.odom.th);

							//first, we'll publish the transform over tf
							geometry_msgs::TransformStamped odom_trans;
							odom_trans.header.stamp = current_time;
							odom_trans.header.frame_id = odom_id;
							odom_trans.child_frame_id = base_link_id;

							odom_trans.transform.translation.x = recv_buf.odom.x;
							odom_trans.transform.translation.y = recv_buf.odom.y;
							odom_trans.transform.translation.z = 0.0;
							odom_trans.transform.rotation = odom_quat;

							//send the transform
							odom_broadcaster.sendTransform(odom_trans);

							//next, we'll publish the odometry message over ROS
							nav_msgs::Odometry odom;
							odom.header.stamp = current_time;
							odom.header.frame_id = "odom";

							//set the position
							odom.pose.pose.position.x = recv_buf.odom.x;
							odom.pose.pose.position.y = recv_buf.odom.y;
							odom.pose.pose.position.z = 0.0;
							odom.pose.pose.orientation = odom_quat;

							//set the velocity
							odom.child_frame_id = "base_link";
							odom.twist.twist.linear.x = recv_buf.odom.vx;
							odom.twist.twist.linear.y = recv_buf.odom.vy;
							odom.twist.twist.angular.z = recv_buf.odom.vth;

							//publish the message
							odom_pub.publish(odom);
							break;
					}
				}
			}
			else
			{
				// try and open the serial port
				try
				{
					ser.setPort(port);
					ser.setBaudrate(115200);
					serial::Timeout to = serial::Timeout::simpleTimeout(1000);
					ser.setTimeout(to);
					ser.open();
				}
				catch (serial::IOException& e)
				{
					ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
					ros::Duration(5).sleep();
				}

				if(ser.isOpen())
				{
					ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
				}
			}
		}
		catch (serial::IOException& e)
		{
			ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
			ser.close();
		}
		ros::spinOnce();
		r.sleep();
	}
}
