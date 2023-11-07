
#include "pwm_servo.h"
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
PWM_Servo::PWM_Servo(int addr) : Adafruit_PWMServoDriver(addr)
{
    Adafruit_PWMServoDriver::begin();
    Adafruit_PWMServoDriver::setPWMFreq(50);
    delay(10);
    Wire.begin();
    Wire.setClock(400000);
    // initalize other variables
    this->joint_servo_iterator = this->joint_servo.begin();
}
PWM_Servo::PWM_Servo() : PWM_Servo::PWM_Servo(0x40) {}

void PWM_Servo::update_pwm(std::string joint_name, float rad)
{
    joint_servo_map_it it = this->joint_servo.find(joint_name);
    if (it == this->joint_servo.end()) return;
    joint_servo[it->first].pwm = radiansToPWM(rad);
}

void PWM_Servo::spinOnce()
{
    if ((this->curr = millis()) - prev > 30)
    {
        servo_info_t info = this->joint_servo_iterator->second;
        this->setPWM(info.pin, 0, info.pwm);
        // loop through the joints
        if (++this->joint_servo_iterator == this->joint_servo.end())
            this->joint_servo_iterator = this->joint_servo.begin();
    }
}

int PWM_Servo::radiansToPWM(float rad)
{
    if (rad >= PWM_Servo::PI_OVER_2) {
        return this->pulse_min;
    }
    if (rad <= -PWM_Servo::PI_OVER_2) {
        return this->pulse_max;
    }
    return (int) round(mapfloat(rad, -PWM_Servo::PI_OVER_2, PWM_Servo::PI_OVER_2, this->pulse_min, this->pulse_max));
}

void PWM_Servo::debug_info()
{
    static int curr = 0, prev = 0;
    if ((curr = millis()) - prev > 1000) {
        for (joint_servo_map_it it = this->joint_servo.begin(); it != this->joint_servo.end(); it++) {
            Serial.print("name: ");
            Serial.print(it->first.c_str());
            Serial.print(" pwm: ");
            Serial.println(it->second.pwm);
        }
    }
}