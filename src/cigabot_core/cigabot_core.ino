#include "cigabot_core_config.h"

void setup() {
  // put your setup code here, to run once:

  nh.initNode();
  nh.subscribe(cmd_vel_sub);

  nh.advertise(odom_pub);


}

void loop() {
  // put your main code here, to run repeatedly:
  n = digitalRead(F_R_ENCA);
  if ((F_R_ENCALAST == LOW) && (n == HIGH)) {
    if (digitalRead(F_R_ENCB) == LOW) {
      encoder0_pos--;
    } else {
      encoder0_pos++;
    }
    Serial.println (encoder0_pos);
    encoder0.data = encoder0_pos;
    pub_encoder0.publish( &encoder0 );
  } 
  F_R_ENCALAST = n;



}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{

}

bool calcOdometry(double diff_time)
{
  return true;
}



