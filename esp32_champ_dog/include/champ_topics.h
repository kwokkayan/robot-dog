#ifndef CHAMP_TOPICS_H
#define CHAMP_TOPICS_H
#include <ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
// TODO: add LiDAR support
void joint_trajectory_callback(const trajectory_msgs::JointTrajectory &champ_joint_trajectory);
sensor_msgs::JointState champ_joint_state;
sensor_msgs::Imu champ_imu;
// TODO: change name
ros::Subscriber<trajectory_msgs::JointTrajectory> joint_trajectory_sub("joint_group_position_controller/command", &joint_trajectory_callback);
ros::Publisher joint_states_pub("joint_states", &champ_joint_state);
ros::Publisher imu_pub("imu/data", &champ_imu);
// echo
void echo(const std_msgs::String &echo_msg);
ros::Subscriber<std_msgs::String> sub("echo", &echo);
#endif