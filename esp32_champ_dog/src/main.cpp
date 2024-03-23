#include "Config.h"
#include <Arduino.h>
#include <std_msgs/String.h>
#include <map>
#include "ros.h"
#include "champ_topics.h"
#include "pwm_servo.h"
#include "mpu_9250.h"
#ifdef ENABLE_WIFI
#include <WiFi.h>
IPAddress ip;
#endif
ros::NodeHandle nh;
PWM_Servo Servo;
MPU_9250 mpu;

void joint_trajectory_callback(const trajectory_msgs::JointTrajectory &msg)
{
  if (msg.points_length > 0)
  {
    for (int i = 0; i < msg.joint_names_length; i++)
    {
      Servo.update_pwm(msg.joint_names[i], msg.points[0].positions[i]);
    }
  }
}

void handleMsgs()
{
  static u_int64_t curr = millis(), prev = 0;
  if ((curr = millis()) - prev > 5)
  {
    prev = curr;
    champ_imu = mpu.composeMsg();
    champ_imu.header.stamp = nh.now();
    imu_pub.publish(&champ_imu);
  }
}

void setup()
{
  // put your setup code here, to run once:
#ifdef ENABLE_WIFI
  Serial.begin(115200);
  WiFi.begin("iinnii", "qwerasdf");
  while (!WiFi.isConnected())
  {
    Serial.println(WiFi.status());
  }
  ip.fromString(ROS_SERIAL_IP);
  nh.getHardware()->setConnection(ip, ROS_SERIAL_PORT);
#else
#ifdef USE_BLUETOOTH
  Serial.begin(115200);
#else
  Serial2.begin(115200);
  nh.getHardware()->setBaud(1000000);
#endif
#endif
  nh.initNode();
  // TODO: new function
  nh.subscribe(joint_trajectory_sub);
  nh.advertise(imu_pub);
  Servo.begin_hw();
  mpu.begin_hw();
}

void loop()
{
  // put your main code here, to run repeatedly:
  mpu.spinOnce();
  handleMsgs();
  nh.spinOnce();
}