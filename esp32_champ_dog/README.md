# ESP32_CHAMP_DOG
## File Structure
- `include`: header files and macro configs
    - `Config.h`: choose the protocol for ROS serial (Wifi, Bluetooth SSP, and Serial are available)
- `lib`: custom libraries
    - `mpu_9250`: IMU wrapper class
    - `pwm_servo`: PWM servo wrapper class
        - `pwm_servo_offset.h`: put joint offsets after calibration here
- `src`: main files
    - `BluetoothSerial.cpp`: configure TX/RX queue sizes
    - `main.cpp`: main firmware
## How to run
1. Choose the protocol used in `Config.h`:
    - Uncomment `#define ENABLE_WIFI` to enable wifi
    - Uncomment `#define USE_BLUETOOTH` to enable bluetooth
    - Comment both to use Serial
2. Configure ros serial buffers (`ros.h`)and bluetooth buffers (`BluetoothSerial.cpp`)
3. Using Platform IO, build and flash.
4. Calibrate joints using calibration mode and adjust offsets in `pwm_servo_offset.h`
## Dependencies
- `frankjoshua/Rosserial Arduino Library@^0.9.1`
- `adafruit/Adafruit PWM Servo Driver Library@^3.0.1`
- `Wire`
- `SPI`
- `kwokkayan/SparkFun MPU-9250 Digital Motion Processing (DMP) Arduino Library (For ESP32)@^1.0.1`: forked from [here](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library)
## Bluetooth commands
- `sudo hcitool cc <MAC>`
- `sudo hcitool auth <MAC>`
- `sudo rfcomm bind rfcomm1 <MAC>`
- `sudo rfcomm connect /dev/rfcomm1 <MAC> 1`
## How to get absoulte orientation
1. Before powering on, face the robot to the EAST to calibrate to ENU coordinates.