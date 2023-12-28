#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include "esp32_ros_controllers/esp32_ros_controllers.h"
esp32_ros_controllers::ESP32Champ *esp32Champ;
controller_manager::ControllerManager *cm;

void imu_msg_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
    esp32Champ->angular_velocity[0] = imu->angular_velocity.x;
    esp32Champ->angular_velocity[1] = imu->angular_velocity.y;
    esp32Champ->angular_velocity[2] = imu->angular_velocity.z;

    esp32Champ->linear_acceleration[0] = imu->linear_acceleration.x;
    esp32Champ->linear_acceleration[1] = imu->linear_acceleration.y;
    esp32Champ->linear_acceleration[2] = imu->linear_acceleration.z;

    esp32Champ->orientation[0] = imu->orientation.x;
    esp32Champ->orientation[1] = imu->orientation.y;
    esp32Champ->orientation[2] = imu->orientation.z;
    esp32Champ->orientation[3] = imu->orientation.w;
    for (int i = 0; i < 9; i++) {
        esp32Champ->angular_velocity_covariance[i] = imu->angular_velocity_covariance[i];
        esp32Champ->orientation_covariance[i] = imu->orientation_covariance[i];
        esp32Champ->linear_acceleration_covariance[i] = imu->linear_acceleration_covariance[i];
    }
}

void loop(const ros::TimerEvent& event)
{
    ros::Duration dt = event.current_expected - event.last_expected;
    esp32Champ->read(event.current_expected, dt);
    cm->update(event.current_expected, dt);
    esp32Champ->write(event.current_expected, dt);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "esp32_champ_controller");
    ros::NodeHandle nh;
    esp32Champ = new esp32_ros_controllers::ESP32Champ(nh);
    cm = new controller_manager::ControllerManager(esp32Champ, nh);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/hw_imu_data", 100, imu_msg_callback);
    // params
    double frequency = 200;
    nh.param<double>("/hw_controller/loop_rate", frequency, 200);
    ros::AsyncSpinner spinner(3);
    spinner.start();
    // ROS loop
    ros::Timer loop_timer = nh.createTimer(ros::Duration(1 / frequency), &loop);
    while (ros::ok()) {}
    // clean up
    spinner.stop();
    loop_timer.stop();
    nh.shutdown();
}