#include "pwm_servo.h"
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

PWM_Servo::PWM_Servo(int addr) : Adafruit_PWMServoDriver(addr)
{
    // initalize other variables
    this->joint_servo_iterator = this->joint_servo.begin();
}

PWM_Servo::PWM_Servo() : PWM_Servo::PWM_Servo(I2C_ADDR) {}

void PWM_Servo::begin_hw()
{
    this->begin();
    this->setPWMFreq(50);
    delay(10);
    Wire.begin();
    Wire.setClock(400000);
}

void PWM_Servo::update_pwm(std::string joint_name, float rad)
{
    joint_servo_map_it it = this->joint_servo.find(joint_name);
    if (it == this->joint_servo.end())
        return;
    // TODO: calculate offsets and inverse
    joint_servo[it->first].pwm = radiansToPWM(rad - PI_OVER_2);
}

void PWM_Servo::spinOnce()
{
    if ((this->curr = millis()) - this->prev > 30)
    {
        this->prev = this->curr;
        servo_info_t info = this->joint_servo_iterator->second;
        this->setPWM(info.pin, 0, info.pwm);
        // loop through the joints
        if (++this->joint_servo_iterator == this->joint_servo.end())
            this->joint_servo_iterator = this->joint_servo.begin();
    }
}

int PWM_Servo::radiansToPWM(float rad)
{
    // TODO: set limits
    // if (rad >= PWM_Servo::PI_OVER_2) {
    //     return this->pulse_min;
    // }
    // if (rad <= -PWM_Servo::PI_OVER_2) {
    //     return this->pulse_max;
    // }
    return (int)round(mapfloat(rad, -PWM_Servo::PI_OVER_2, PWM_Servo::PI_OVER_2, this->pulse_min, this->pulse_max));
}

void PWM_Servo::debug_info()
{
    static int curr = 0, prev = 0;
    if ((curr = millis()) - prev > 1000)
    {
        prev = curr;
        for (joint_servo_map_it it = this->joint_servo.begin(); it != this->joint_servo.end(); it++)
        {
            Serial.print("name: ");
            Serial.print(it->first.c_str());
            Serial.print(" pwm: ");
            Serial.println(it->second.pwm);
        }
    }
}