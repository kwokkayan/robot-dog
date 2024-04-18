#include <Adafruit_PWMServoDriver.h> //https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Wire.h>
#include <math.h>
#include <map>
#include "pwm_servo_offset.h"
#ifndef PWM_SERVO_H
#define PWM_SERVO_H
#define I2C_ADDR 0x41
// #define I2C_ADDR 0x70
typedef struct {
    float min_angle;
    float max_angle;
    int pulse_min;
    int pulse_max;
} map_info_t;

typedef struct
{
    int pin;
    int pwm;
    float angleOffset; // offset in angles
    float scalar; // scale the pwm signal
    map_info_t map_info;
} servo_info_t;

typedef std::map<std::string, servo_info_t> joint_servo_map;
typedef std::map<std::string, servo_info_t>::iterator joint_servo_map_it;
// util functions
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
/**
 * This class encapsulates the 12 PWM servos.
 * It converts the angle (-PI to PI) to PWM cycles
 * When joints are all straight => 0 radians
 * 90 degree leg => -PI/2 for lower leg, 0 radians for others
 * ======
 * PWM definitions:
 * 90 degree leg = 0 pwm
 */
class PWM_Servo : public Adafruit_PWMServoDriver
{
public:
    PWM_Servo(int);
    PWM_Servo();
    void begin_hw();
    /**
     * This function updates the pwm value for the pin.
     */
    void update_pwm(std::string, float);
    void debug_info();
    void debug_pwm();

private:
    const float PI_OVER_2 = PI / 2;
    map_info_t map_info = { -PI_OVER_2, PI_OVER_2, 105, 500 };
    map_info_t TD8125_info = { -PI_OVER_2, PI_OVER_2, 102, 512 };
    joint_servo_map joint_servo = {
        // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
        {"lf_hip_joint", {5, 0, LF_HIP_JOINT_OFFSET, 1.0f, TD8125_info}},
        {"lf_upper_leg_joint", {6, 0, LF_UPPER_LEG_JOINT_OFFSET, 1.0f, TD8125_info}},
        {"lf_lower_leg_joint", {7, 0, LF_LOWER_LEG_JOINT_OFFSET, -1.0f, TD8125_info}},
        // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
        {"rf_hip_joint", {10, 0, RF_HIP_JOINT_OFFSET, 1.0f, TD8125_info}},
        {"rf_upper_leg_joint", {9, 0, RF_UPPER_LEG_JOINT_OFFSET, -1.0f, TD8125_info}},
        {"rf_lower_leg_joint", {8, 0, RF_LOWER_LEG_JOINT_OFFSET, 1.0f, TD8125_info}},
        // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
        {"lh_hip_joint", {2, 0, LH_HIP_JOINT_OFFSET, -1.0f, TD8125_info}},
        {"lh_upper_leg_joint", {1, 0, LH_UPPER_LEG_JOINT_OFFSET, 1.0f, TD8125_info}},
        {"lh_lower_leg_joint", {0, 0, LH_LOWER_LEG_JOINT_OFFSET, -1.0f, TD8125_info}},
        // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
        {"rh_hip_joint", {13, 0, RH_HIP_JOINT_OFFSET, -1.0f, TD8125_info}},
        {"rh_upper_leg_joint", {14, 0, RH_UPPER_LEG_JOINT_OFFSET, -1.0f, TD8125_info}},
        {"rh_lower_leg_joint", {15, 0, RH_LOWER_LEG_JOINT_OFFSET, 1.0f, TD8125_info}},
    };
    // set 90 degrees as constraint
    int radiansToPWM(float, map_info_t);
    // spinOnce variables
    u_long curr = 0, prev = 0;
    joint_servo_map_it joint_servo_iterator;
};
#endif