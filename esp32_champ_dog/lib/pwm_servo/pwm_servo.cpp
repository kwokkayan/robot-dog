#include "pwm_servo.h"
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (x <= in_min) {
        return out_min;
    }
    if (x >= in_max) {
        return out_max;
    }
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
    servo_info_t *info = &joint_servo[it->first];
    info->pwm = radiansToPWM((rad + info->angleOffset) * info->scalar, info->map_info);
    this->setPWM(info->pin, 0, info->pwm);
}

void PWM_Servo::spinOnce()
{
    servo_info_t info = this->joint_servo_iterator->second;
    this->setPWM(info.pin, 0, info.pwm);
    // loop through the joints
    if (++this->joint_servo_iterator == this->joint_servo.end())
        this->joint_servo_iterator = this->joint_servo.begin();
}

int PWM_Servo::radiansToPWM(float rad, map_info_t info)
{
    return (int)round(mapfloat(rad, info.min_angle, info.max_angle, info.pulse_min, info.pulse_max));
}

void PWM_Servo::debug_info()
{
    static int curr = 0, prev = 0;
    if ((curr = millis()) - prev > 1000)
    {
        prev = curr;
        for (joint_servo_map_it it = this->joint_servo.begin(); it != this->joint_servo.end(); it++)
        {
            Serial2.print("name: ");
            Serial2.print(it->first.c_str());
            Serial2.print(" pwm: ");
            Serial2.println(it->second.pwm);
        }
    }
}

void PWM_Servo::debug_pwm()
{
  int pin;
  int val;
  String s;
  bool enteredPin = false;
  while (true)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      s += c;
      Serial.print(c);
      if (c == '\n') {
        if (enteredPin) {
          val = s.toInt();
          break;
        } else {
          pin = s.toInt();
          s = "";
          enteredPin = true;
        }
      }
    }
  }
  this->setPWM(pin, 0, val);
}