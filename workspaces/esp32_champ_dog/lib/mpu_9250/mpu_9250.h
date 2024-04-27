#include <SparkFunMPU9250-DMP.h>
#include <sensor_msgs/Imu.h>
#ifndef MPU_9250_H
#define MPU_9250_H
#define CALIBRATE_SAMPLE_SIZE 10000
class MPU_9250 : public MPU9250_DMP
{
public:
    MPU_9250();
    void begin_hw();
    void spinOnce();
    sensor_msgs::Imu composeMsg();
    void printIMUData();
    void getAccelOffset();
private:
    sensor_msgs::Imu out;
};
#endif