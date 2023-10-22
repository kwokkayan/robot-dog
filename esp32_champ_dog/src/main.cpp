#include <Config.h>
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "champ_topics.h"

ros::NodeHandle nh;
char *joint_names[] = {(char *)"lf_hip_joint", (char *)"lf_upper_leg_joint", (char *)"lf_lower_leg_joint", (char *)"rf_hip_joint", (char *)"rf_upper_leg_joint",
                       (char *)"rf_lower_leg_joint", (char *)"lh_hip_joint", (char *)"lh_upper_leg_joint", (char *)"lh_lower_leg_joint", (char *)"rh_hip_joint",
                       (char *)"rh_upper_leg_joint", (char *)"rh_lower_leg_joint"};
float position[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
unsigned long curr, prev;
void joint_trajectory_callback(const trajectory_msgs::JointTrajectory &champ_joint_trajectory)
{
}

void echo(const std_msgs::String &echo_msg)
{
  nh.loginfo(echo_msg.data);
}

void setup()
{
  // put your setup code here, to run once:
  nh.initNode();
  // TODO: new function
  nh.subscribe(sub);
  nh.subscribe(joint_trajectory_sub);
  nh.advertise(joint_states_pub);
  nh.advertise(imu_pub);
  // test driver code
  champ_joint_state.position_length = 12;
  champ_joint_state.velocity_length = 0;
  champ_joint_state.effort_length = 0;
  champ_joint_state.name_length = 12;
  champ_joint_state.name = joint_names;
  champ_joint_state.position = position;
  // timer
  prev = 0;
  curr = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  if ((prev = millis()) - curr > 1000)
  {
    curr = prev;
    champ_joint_state.header.stamp = nh.now();
    joint_states_pub.publish(&champ_joint_state);
  }
  delay(1);
}
