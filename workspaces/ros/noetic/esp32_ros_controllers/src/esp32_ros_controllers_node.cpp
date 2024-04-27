#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include "esp32_ros_controllers/esp32_ros_controllers.h"
esp32_ros_controllers::ESP32Champ *esp32Champ;
controller_manager::ControllerManager *cm;

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