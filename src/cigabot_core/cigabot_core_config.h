#ifndef CIGABOT3_CORE_CONFIG_H_
#define CIGABOT3_CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define F_R_PWM 3 //pwm to control speed
#define F_R_IN1 4 //in1 from driver // pins to control direction
#define F_R_IN2 5 //in2 from driver // pins to control direction
#define F_R_ENCA 6 // encoder
#define F_R_ENCB 7 // encoder

#define F_L_PWM 8 //pwm to control speed
#define F_L_IN1 9 //in1 from driver // pins to control direction
#define F_L_IN2 10 //in2 from driver // pins to control direction
#define F_L_ENCA 11 // encoder
#define F_L_ENCB 12 // encoder

int encoder0_pos = 0;
int F_R_ENCALAST = LOW;
int n = LOW;

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

// Function prototypes
bool calcOdometry(double diff_time);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);


/*******************************************************************************
* Publisher
*******************************************************************************/
nav_msgs::Odometry odom;
std_msgs::UInt16 encoder0;
ros::Publisher odom_pub("odom", &odom);
ros::Publisher pub_encoder0("encoder0", &encoder0);

#endif // TURTLEBOT3_CORE_CONFIG_H_
