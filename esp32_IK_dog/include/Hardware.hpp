#ifndef __HARDWARE__
#define __HARDWARE__
#include "Config.hpp"
#include <Adafruit_PWMServoDriver.h> //https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include "datatypes.h"
#include <exception>
#include "cc.hpp"
#include "lex.cc.hpp"
extern int matrix[4][3];
extern int val_min;
extern int val_max;
/*
  #include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
*/
class Hardware : public Adafruit_PWMServoDriver {

  private:
    /*
      ==============================
      HARDWARE - SERVO PARAMETERS
      ==============================
    */
    const int s_output[4][3] = {
      {4, 5, 6},    // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
      {0, 1, 2},    // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
      {15, 14, 13}, // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
      {11, 10, 9}   // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
    };

    const int pulse_min = 105;
    const int pulse_max = 500;
    int s_offset_pulse[4][3] = {
      {5, -5, 0},
      {-10, 0, -10},
      {-5, 0, 20},
      {0, 0, 5}
    };
    // int s_offset_pulse[4][3] = {
    //   {-3, 20, 10}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
    //   {-12, 20, -2}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
    //   {-7,  -10, -10}, // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
    //   {0, -15, -20}  // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
    // };
    // int s_offset_pulse[4][3] = {
    //   {9, 0, -10}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
    //   {-12, 0, -22}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
    //   {-7,  5, 22}, // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
    //   {0, -2, 10}  // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
    // };
    //  int s_offset_pulse[4][3] = {
    //   {10, 0, 0}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
    //   {-12, 0, 0}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
    //   {5, 0, 0}, // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
    //   {-8, 0, 0}  // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
    // };
    // ## pulse offset - offset value to adjust servo to center 90°


  //  int s_offset_pulse[4][3] = {
  //    { -10, -10, -15},
  //    {10, 10, 0},      
  //    {0, 10, 35}, 
  //    {0, 10, 30}
  //  };

    // big red
  //  int s_offset_pulse[4][3] = {
  //     {10, 20, 30}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right back
  //     {0, 70, 0}, // ## {shoulder chnl, upper chnl, lower chnl} robot's right front
  //     {0, -20, -20}, // ## {shoulder chnl, upper chnl, lower chnl} robot's left front
  //     {-20, 0, -30}  // ## {shoulder chnl, upper chnl, lower chnl} robot's left back
  //   };    
    
    const int s_optinv[4][3] = {
      {0, 0, 0}, // ## {dir, dir, dir}
      {1, 0, 0}, // ## {dir, dir, dir}
      {0, 1, 1}, // ## {dir, dir, dir}
      {1, 1, 1}  // ## {dir, dir, dir}
    };
    const int d_constraint_min[3] { -70, 20, 40}; // ## {deg, deg, deg}
    const int d_constraint_max[3] {70, 110, 150}; // ## {deg, deg, deg}


    //
    boolean attached = false;
  public:
    Hardware(int addr): Adafruit_PWMServoDriver(addr) {}
    Hardware(): Hardware(0x40) {}

    /*
      ::: SETUP :::
    */
    void init_hardware()
    {
      Adafruit_PWMServoDriver::begin();
      Adafruit_PWMServoDriver::setPWMFreq(50);
      delay(10);
      //init_mpu();
      Wire.begin();
      Wire.setClock(400000);
      pinMode(OE_PIN, OUTPUT);
      attach();
    }

    void attach() {
      const int right_back_leg = 0;
      const int right_front_leg = 1;
      const int left_front_leg = 2;
      const int left_back_leg = 3;      
      int pulse = map(90, 0, 180, pulse_min, pulse_max);
      int legs[] = {
        right_back_leg,
        left_back_leg,
        right_front_leg,
        left_front_leg
      };
      if (attached) return;
      digitalWrite(OE_PIN, LOW);
      for (int joint = 0; joint < 3; joint++) {
        for (int i = 0; i < 4; i++) {
          int channel = s_output[legs[i]][joint];
          int offset = s_offset_pulse[legs[i]][joint];
          setPWM(channel, 0, pulse + offset);
          delay(30);
        }
      }
      attached = true;
    }

    void detach() {
      if (!attached) return;
      digitalWrite(OE_PIN, HIGH);
      attached = false;
    }

    void set_leg(int leg, datatypes::Rotator rot)
    {
      set_joint(leg, 0, rot.yaw);
      set_joint(leg, 1, rot.pitch);
      set_joint(leg, 2, rot.roll);
    }

    void set_servo(int leg, int joint, float pulse)
    {
      int _num = s_output[leg][joint];
      setPWM(_num, 0, pulse);
    }

    void set_offset_with_command()
    {
      val_min = -100;
      val_max = 100;
      for (int i = 0; i < 4; i++) {
        matrix[i][0] = s_offset_pulse[i][0];
        matrix[i][1] = s_offset_pulse[i][1];
        matrix[i][2] = s_offset_pulse[i][2];
      }
      String s;
      while (true) {
        if (Console.available()) {
          char c = Serial.read();
          s += c;
          Serial.print(c);
          if (c == '\n') break;
        }
      }
      char *exp = (char *) malloc(sizeof(char) *(s.length() + 1));
      strcpy(exp, s.c_str());
      yy_scan_string(exp);
      int status = yyparse();
      free(exp);
      for (int i = 0; i < 4; i++) {
        if (status == 0) {
          s_offset_pulse[i][0] = matrix[i][0];
          s_offset_pulse[i][1] = matrix[i][1];
          s_offset_pulse[i][2] = matrix[i][2];
        }
      }
      print_offset();
      detach();
      attach();
    }

  private:
    void print_offset()
    {
      for (int i = 0; i < 4; i++)
        Console.printf("[ %d, %d, %d ]\n", s_offset_pulse[i][0], s_offset_pulse[i][1], s_offset_pulse[i][2]);
    }
    void set_joint(int leg, int joint, float deg)
    {
      int _min = pulse_min;
      int _max = pulse_max;
      int _inv = s_optinv[leg][joint];
      int _minC = d_constraint_min[joint];
      int _maxC = d_constraint_max[joint];

      if (deg < _minC)
        deg = _minC;
      else if (deg > _maxC)
        deg = _maxC;
      int _num = s_output[leg][joint];
      if (_inv == 0)
        setPWM(_num, 0, map(deg, _minC, _maxC, _min, _max) + s_offset_pulse[leg][joint]);
      else if (_inv == 1)
        setPWM(_num, 0, map(deg , _minC, _maxC, _max, _min) + s_offset_pulse[leg][joint]);
    }
    /*
      == == == == == == == == == == == == == == ==
      HARDWARE - MPU VARIABLES
      == == == == == == == == == == == == == == ==
      /

      //uint16_t packetSize;    // expected DMP packet size (default 42 bytes)
      //uint8_t fifoBuffer[64]; // FIFO storage buffer

      //Quaternion q;           // [w, x, y, z]         quaternion container
      //float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
      //VectorFloat gravity;    // [x, y, z]            gravity vector

      MPU6050 mpu;
   
      ::: [Gyroscope/Accelerometer Sensor] FUNCTIONS :::
    */

    void update_mpu_data()
    {
      /*if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        _sRotation = {ypr[0] * 80 / M_PI,
                      -ypr[1] * 80 / M_PI,
                      ypr[2] * 80 / M_PI
                     };

       if (DEBUG == 0) {
          Console.print(ypr[0] * 60 / M_PI);
          Console.print("/");
          Console.print(-ypr[1] * 60 / M_PI);
          Console.print("/");
          Console.println(ypr[2] * 60 / M_PI);
          }
        }*/
    }
  public:
    void handle_hardware()
    {
      //update_mpu_data();
    }


    void init_mpu()
    {
      /*mpu.initialize();
        uint8_t dmp_s = mpu.dmpInitialize();

        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);

        if (dmp_s == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
        if (DEBUG == 1)
          Console.println(":ERROR[" + String(dmp_s) + "]");
        while (1); // ::pause_sketch::
        }*/
    }
};
#endif
