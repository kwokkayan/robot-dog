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
  this->setLPF(42);
  this->setSampleRate(1000);
  this->dmpBegin(DMP_FEATURE_6X_LP_QUAT |        // Enable 6-axis quat
                     DMP_FEATURE_GYRO_CAL |      // Use gyro calibration
                     DMP_FEATURE_SEND_CAL_GYRO | // send angular velocity
                     DMP_FEATURE_SEND_RAW_ACCEL, // send acceleration
                 20);                            // Set DMP FIFO rate to 20 Hz
  delay(100);
  this->calibrateAccel();
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  for (int i = 0; i < 9; i++)
  {
    out.orientation_covariance[i] = 0;
    out.angular_velocity_covariance[i] = 0;
    out.linear_acceleration_covariance[i] = 0;
  }
  // From datasheet:
  // gyro Total RMS noise = 0.1 degree/s
  // accel Total RMS noise = 8 mG
  out.angular_velocity_covariance[0] = 0.1 * RadianConst;
  out.angular_velocity_covariance[0] *= out.angular_velocity_covariance[0];
  out.angular_velocity_covariance[4] = out.angular_velocity_covariance[0];
  out.angular_velocity_covariance[8] = out.angular_velocity_covariance[0];

  out.linear_acceleration_covariance[0] = 0.008 * G;
  out.linear_acceleration_covariance[0] *= out.linear_acceleration_covariance[0];
  out.linear_acceleration_covariance[4] = out.linear_acceleration_covariance[0];
  out.linear_acceleration_covariance[8] = out.linear_acceleration_covariance[0];
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
      // printIMUData();
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

sensor_msgs::Imu MPU_9250::composeMsg()
{
  out.orientation.w = this->calcQuat(this->qw);
  out.orientation.x = this->calcQuat(this->qx);
  out.orientation.y = this->calcQuat(this->qy);
  out.orientation.z = this->calcQuat(this->qz);

  out.angular_velocity.x = this->calcDegreeToRadians(this->calcGyro(this->gx));
  out.angular_velocity.y = this->calcDegreeToRadians(this->calcGyro(this->gy));
  out.angular_velocity.z = this->calcDegreeToRadians(this->calcGyro(this->gz));

  out.linear_acceleration.x = this->calcGToMps(this->calcAccel(this->ax));
  out.linear_acceleration.y = this->calcGToMps(this->calcAccel(this->ay));
  out.linear_acceleration.z = this->calcGToMps(this->calcAccel(this->az));

  return out;
}

void MPU_9250::getAccelOffset()
{
  long accel_bias[3] = {0, 0, 0};
  mpu_read_6500_accel_bias(accel_bias);
  float accelX = this->calcAccel(this->ax);
  float accelY = this->calcAccel(this->ay);
  float accelZ = this->calcAccel(this->az);
  SerialPort.print(this->ax);
  SerialPort.print(" ");
  SerialPort.print(this->ay);
  SerialPort.print(" ");
  SerialPort.println(this->az);
  SerialPort.print(accelX);
  SerialPort.print(" ");
  SerialPort.print(accelY);
  SerialPort.print(" ");
  SerialPort.println(accelZ);
  SerialPort.print(accel_bias[0]);
  SerialPort.print(" ");
  SerialPort.print(accel_bias[1]);
  SerialPort.print(" ");
  SerialPort.println(accel_bias[2]);
  SerialPort.println();
  delay(100);
}

void MPU_9250::calibrateAccel()
{
  long accel_bias[3] = {0, 0, 0};
  uint16_t accelsensitivity = 16384; // = 16384 LSB/g
  for (int i = 0; i < CALIBRATE_SAMPLE_SIZE; i++) {
    this->dmpUpdateFifo();
    accel_bias[0] += (long) this->ax;
    accel_bias[1] += (long) this->ay;
    accel_bias[2] += (long) this->az - accelsensitivity;
  }
  accel_bias[0] /= CALIBRATE_SAMPLE_SIZE;
  accel_bias[1] /= CALIBRATE_SAMPLE_SIZE;
  accel_bias[2] /= CALIBRATE_SAMPLE_SIZE;

  accel_bias[0] /= 8;
  accel_bias[1] /= 8;
  accel_bias[2] /= 8;
  mpu_set_accel_bias_6500_reg(accel_bias);
}

float MPU_9250::calcDegreeToRadians(float v)
{
  return v * RadianConst;
}

float MPU_9250::calcGToMps(float v)
{
  return v * G;
}