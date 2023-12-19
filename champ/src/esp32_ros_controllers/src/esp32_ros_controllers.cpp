#include <ros/ros.h>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>
#include "esp32_ros_controllers/esp32_ros_controllers.h"
namespace esp32_ros_controllers
{
    ESP32Champ::ESP32Champ(ros::NodeHandle& nh)
    {
        this->nh = nh;
        this->hw_joint_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("hw_joint_trajectory", 100);
        this->nh.getParam("/hw_controller/joints", this->joint_names);
        this->nh.getParam("/hw_controller/name", this->name);
        this->nh.getParam("/hw_controller/frame_id", this->frame_id);
        if (this->joint_names.size() <= 0) {
            ROS_ERROR("No joints found in .yaml! Exiting...");
            exit(-1);
        }
        // initialize controllers
        this->cmd = (double*)malloc(sizeof(double) * this->joint_names.size());
        this->pos = (double*)malloc(sizeof(double) * this->joint_names.size());
        this->vel = (double*)malloc(sizeof(double) * this->joint_names.size());
        this->eff = (double*)malloc(sizeof(double) * this->joint_names.size());
        this->angular_velocity = (double*)malloc(sizeof(double) * 3);
        this->linear_acceleration = (double*)malloc(sizeof(double) * 3);
        this->orientation = (double*)malloc(sizeof(double) * 4);
        this->angular_velocity_covariance = (double*)malloc(sizeof(double) * 9);
        this->linear_acceleration_covariance = (double*)malloc(sizeof(double) * 9);
        this->orientation_covariance = (double*)malloc(sizeof(double) * 9);
        // register interface
        for (int i = 0; i < this->joint_names.size(); i++) {
            hardware_interface::JointStateHandle state_handle(joint_names.at(i), &pos[i], &vel[i], &eff[i]);
            jnt_state_interface.registerHandle(state_handle);
        }
        registerInterface(&jnt_state_interface);
        for (int i = 0; i < this->joint_names.size(); i++) {
            hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names.at(i)), &cmd[i]);
            jnt_pos_interface.registerHandle(pos_handle);
        }
        registerInterface(&jnt_pos_interface);

        for (int i = 0; i < 9; i++) {
            angular_velocity_covariance[i] = 0;
            orientation_covariance[i] = 0;
            linear_acceleration_covariance[i] = 0;
        }

        hardware_interface::ImuSensorHandle imu_handle(name, frame_id, orientation, 
            orientation_covariance, angular_velocity, angular_velocity_covariance, 
            linear_acceleration, linear_acceleration_covariance);
        imu_sensor_interface.registerHandle(imu_handle);
        registerInterface(&imu_sensor_interface);
    }

    ESP32Champ::~ESP32Champ() {
        free(this->cmd);
        free(this->pos);
        free(this->vel);
        free(this->eff);
        free(this->angular_velocity);
        free(this->angular_velocity_covariance);
        free(this->linear_acceleration);
        free(this->linear_acceleration_covariance);
        free(this->orientation);
        free(this->orientation_covariance);
    }

    void ESP32Champ::read(const ros::Time& time, const ros::Duration& period)
    {
        // assume the motors work as intended
        for (int i = 0; i < this->joint_names.size(); i++) {
            this->pos[i] = this->cmd[i];
        }
    }

    void ESP32Champ::write(const ros::Time& time, const ros::Duration& period)
    {
        trajectory_msgs::JointTrajectory msg;
        msg.points.resize(1);
        msg.header.stamp = time;
        msg.points.at(0).velocities.resize(0);
        msg.points.at(0).effort.resize(0);
        for (int i = 0; i < this->joint_names.size(); i++) {
            msg.joint_names.push_back(joint_names.at(i));
            msg.points.at(0).positions.push_back(this->cmd[i]);
        }
        this->hw_joint_trajectory_pub.publish(msg);
    }

} // namespace esp32_ros_controllers
