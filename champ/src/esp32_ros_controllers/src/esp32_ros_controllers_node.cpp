#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/console.h>
#include "esp32_ros_controllers/esp32_ros_controllers.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "esp32_champ_controller");
    ros::NodeHandle nh;
    esp32_ros_controllers::ESP32Champ esp32Champ(nh);
    controller_manager::ControllerManager cm(&esp32Champ, nh);
    // params
    double frequency = 10;
    int baud_rate = 115200;
    std::string port = "/dev/ttyUSB0";
    nh.param<double>("/hw_controller/loop_rate", frequency, 10);
    nh.param<int>("/hw_controller/baud_rate", baud_rate, 115200);
    nh.param<std::string>("/hw_controller/port", port, "/dev/ttyUSB0");
    // Async spinner
    ros::AsyncSpinner spinner(3);
    spinner.start();
    // ROS loop
    ros::Rate loop_rate(frequency);
    ros::Time currTime, prevTime = ros::Time::now();
    ros::Duration dt;
    while (ros::ok())
    {
        currTime = ros::Time::now();
        dt = currTime - prevTime;
        prevTime = currTime;
        esp32Champ.read(currTime, dt);
        cm.update(currTime, dt);
        esp32Champ.write(currTime, dt);
        loop_rate.sleep();
    }
    // clean up
    spinner.stop();
    nh.shutdown();
}