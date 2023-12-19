#ifndef CHAMP_TOPICS_H
#define CHAMP_TOPICS_H
#include <ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
// TODO: add LiDAR support
void joint_trajectory_callback(const trajectory_msgs::JointTrajectory &);
sensor_msgs::Imu champ_imu;
// TODO: change name
ros::Subscriber<trajectory_msgs::JointTrajectory> joint_trajectory_sub("/hw_joint_trajectory", &joint_trajectory_callback);
ros::Publisher imu_pub("/hw_imu_data", &champ_imu);
#endif