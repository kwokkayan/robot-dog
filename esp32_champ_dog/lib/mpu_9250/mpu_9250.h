#include <SparkFunMPU9250-DMP.h>
#ifndef MPU_9250_H
#define MPU_9250_H
class MPU_9250 : public MPU9250_DMP
{
public:
    MPU_9250();
    void begin_hw();
    void spinOnce();

private:
    void printIMUData();
    float calcDegreeToRadians(float);
    float calcGToMps(float);
};
#endif