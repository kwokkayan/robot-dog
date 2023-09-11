#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

// ROS PUB/SUB declarations
std_msgs::String str_msg;
ros::Publisher serial("serial", &str_msg);

void echo(const std_msgs::String &echo_msg)
{
  str_msg = echo_msg;
  serial.publish(&str_msg);
}

ros::Subscriber<std_msgs::String> sub("echo", &echo);

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  nh.getHardware();
  nh.initNode();
  nh.advertise(serial);
  nh.subscribe(sub);
}

void loop()
{
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
