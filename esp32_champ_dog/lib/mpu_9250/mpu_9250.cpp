#include "mpu_9250.h"
#define SerialPort Serial
constexpr float RadianConst = PI / 180.0f;
#define G 9.80665
MPU_9250::MPU_9250() : MPU9250_DMP()
{
}

void MPU_9250::begin_hw()
{
  // Call imu.begin() to verify communication and initialize
  if (this->begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  this->dmpBegin(DMP_FEATURE_6X_LP_QUAT |        // Enable 6-axis quat
                     DMP_FEATURE_GYRO_CAL |      // Use gyro calibration
                     DMP_FEATURE_SEND_CAL_GYRO | // send angular velocity
                     DMP_FEATURE_SEND_RAW_ACCEL, // send acceleration
                 10);                            // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void MPU_9250::spinOnce()
{
  // Check for new data in the FIFO
  if (this->fifoAvailable())
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (this->dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      this->computeEulerAngles();
      printIMUData();
    }
  }
}

void MPU_9250::printIMUData()
{
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = this->calcQuat(this->qw);
  float q1 = this->calcQuat(this->qx);
  float q2 = this->calcQuat(this->qy);
  float q3 = this->calcQuat(this->qz);
  // angular velocity
  float gyroX = this->calcDegreeToRadians(this->calcGyro(this->gx));
  float gyroY = this->calcDegreeToRadians(this->calcGyro(this->gy));
  float gyroZ = this->calcDegreeToRadians(this->calcGyro(this->gz));
  // acceleration
  float accelX = this->calcGToMps(this->calcAccel(this->ax));
  float accelY = this->calcGToMps(this->calcAccel(this->ay));
  float accelZ = this->calcGToMps(this->calcAccel(this->az));

  SerialPort.println("Accel: " + String(accelX) + ", " +
                     String(accelY) + ", " + String(accelZ) + " mps^2");
  SerialPort.println("Gyro: " + String(gyroX) + ", " +
                     String(gyroY) + ", " + String(gyroZ) + " rps");
  SerialPort.println("Q: " + String(q0, 4) + ", " +
                     String(q1, 4) + ", " + String(q2, 4) +
                     ", " + String(q3, 4));
  SerialPort.println("R/P/Y: " + String(this->roll) + ", " + String(this->pitch) + ", " + String(this->yaw));
  SerialPort.println("Time: " + String(this->time) + " ms");
  SerialPort.println();
}

float MPU_9250::calcDegreeToRadians(float v)
{
  return v * RadianConst;
}

float MPU_9250::calcGToMps(float v)
{
  return v * G;
}