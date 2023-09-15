
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16.h>

#define PI 3.14159265

class OdometryPublisher
{
	public:
		OdometryPublisher();

	private:
		void encoderCallback(const std_msgs::UInt16::ConstPtr& encoder0);

		ros::NodeHandle nh;
		ros::Subscriber enc_sub;
		ros::Publisher odom_pub;
		tf::TransformBroadcaster odom_broadcaster;
		
		double x, y, th;
		double scale_x;
};


OdometryPublisher::OdometryPublisher()
{
	// intialize integrators
	x = y = th = 0;
	
	// load parameters
    ros::NodeHandle nh_priv("~");
	nh_priv.param("scale_x", scale_x, 0.6283185 / 360.0 / 2.15); // loosely base off of 360 counts per rev, 100mm wheels (which is 0.6283185 / 360.0)

	
	// lets show em what we got
	ROS_INFO_STREAM("param scale_x: " << scale_x);

	// connects subs and pubs
	enc0_sub = nh.subscribe<std_msgs::UInt16>("encoder0", 10, &OdometryPublisher::encoderCallback, this);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

void OdometryPublisher::encoderCallback(const std_msgs::UInt16::ConstPtr& encoder0)
{
	// unpack the encoder message in base_link frame
	ros::Time current_time = ros::Time::now();
	double dx = encoder0->data * scale_x;

	
	// convert to movements in the odom frame
	/*
	if(calibration_mode) {
		x += dx;
		y += dy;
		th += dth;
		//th = fmod(th, 2*PI);
	}
    else {
		x += dx * cos(th) - dy * sin(th);
		y += dx * sin(th) + dy * cos(th);
		th += dth;
		th = fmod(th, 2*PI);
    }
	*/

	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	// first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	if(calibration_mode) odom_trans.transform.translation.z = th; // need some way to check this w/o the quaternion
	else odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	// send the transform
	odom_broadcaster.sendTransform(odom_trans);

	// next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	// set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = dx / dt;
	odom.twist.twist.linear.y = dy / dt;
	odom.twist.twist.angular.z = dth / dt;

	// publish the message
	odom_pub.publish(odom);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");
	OdometryPublisher odom;

	ros::spin();
}