#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
// #define JOINT_NUMS 16
namespace esp32_ros_controllers
{
    class ESP32Champ : public hardware_interface::RobotHW
    {
    public:
        ESP32Champ(ros::NodeHandle&);
        ~ESP32Champ();
        void read(const ros::Time&, const ros::Duration&);
        void write(const ros::Time&, const ros::Duration&);
        double *orientation;
        double *orientation_covariance;
        double *angular_velocity;
        double *angular_velocity_covariance;
        double *linear_acceleration;
        double *linear_acceleration_covariance;
    private:
        ros::NodeHandle nh;
        ros::Publisher hw_joint_trajectory_pub, imu_data_pub;
        std::vector<std::string> joint_names;
        std::string frame_id, name;
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::ImuSensorInterface imu_sensor_interface;
        double *cmd, *pos, *vel, *eff;
    };
}