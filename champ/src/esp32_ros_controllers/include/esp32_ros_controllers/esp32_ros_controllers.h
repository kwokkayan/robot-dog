#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
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
    private:
        ros::NodeHandle nh;
        ros::Publisher hw_joint_trajectory_pub;
        // const std::string joint_names[JOINT_NUMS] = 
        //     { "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", "lf_foot_joint",
        //       "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", "rf_foot_joint", 
        //       "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", "lh_foot_joint",
        //       "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint", "rh_foot_joint" };
        std::vector<std::string> joint_names;
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        double *cmd, *pos, *vel, *eff;
    };
}