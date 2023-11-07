#include "Config.h"
#include <Arduino.h>
#include <std_msgs/String.h>
#include <map>
#include "ros.h"
#include "champ_topics.h"
#include "pwm_servo.h"
ros::NodeHandle nh;
// test code
PWM_Servo Servo;

void joint_trajectory_callback(const trajectory_msgs::JointTrajectory &msg)
{
  for (int i = 0; i < msg.joint_names_length; i++) {
    Servo.update_pwm(msg.joint_names[i], msg.points[0].positions[i]);
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.initNode();
  // TODO: new function
  nh.subscribe(joint_trajectory_sub);
  nh.advertise(imu_pub);
}

void loop()
{
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  Servo.spinOnce();
  Servo.debug_info();
  delay(1);
}
