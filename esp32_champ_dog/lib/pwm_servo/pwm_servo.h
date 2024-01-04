#include <Adafruit_PWMServoDriver.h> //https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Wire.h>
#include <math.h>
#include <map>
#ifndef PWM_SERVO_H
#define PWM_SERVO_H
#define I2C_ADDR 0x41
#define LOWER_OFFSET 1.5707999999999998f
#define SHOULDER_MARGIN 0.05f
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
    joint_servo_map joint_servo = {
        // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
        {"lf_hip_joint", {5, 0, 0.039616040000000075f - SHOULDER_MARGIN, 1.0f}},
        {"lf_upper_leg_joint", {6, 0, 0.08566096000000001f, 1.0f}},
        {"lf_lower_leg_joint", {7, 0, -0.05707607f + LOWER_OFFSET, -1.0f}},
        // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
        //-0.039616039999999964, 0.05717711999999997, -1.59938856
        {"rf_hip_joint", {10, 0, -0.039616039999999964f + SHOULDER_MARGIN, 1.0f}},
        {"rf_upper_leg_joint", {9, 0, 0.05717711999999997f, -1.0f}},
        {"rf_lower_leg_joint", {8, 0, -0.02858856f + LOWER_OFFSET, 1.0f}},
        // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
        {"lh_hip_joint", {3, 0, -0.04764396000000004f - SHOULDER_MARGIN, -1.0f}},
        {"lh_upper_leg_joint", {2, 0, 0.047542880000000176f, 1.0f}},
        {"lh_lower_leg_joint", {1, 0, -0.07142271f + LOWER_OFFSET, -1.0f}},
        // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
        {"rh_hip_joint", {12, 0, -0.09511340000000001f + SHOULDER_MARGIN, -1.0f}},
        {"rh_upper_leg_joint", {13, 0, -0.15226288f, -1.0f}},
        {"rh_lower_leg_joint", {14, 0, 0.02858489f + LOWER_OFFSET, 1.0f}},
    };
    // set 90 degrees as constraint
    int radiansToPWM(float);
    // spinOnce variables
    u_long curr = 0, prev = 0;
    joint_servo_map_it joint_servo_iterator;
};
#endif