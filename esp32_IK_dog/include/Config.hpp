#ifndef __CONFIG__
#define __CONFIG__

//
#define __DEBUG__
//
#define Console Serial


// select the way of control
#define __GOBLE__
//#define __PS2_GAMEPAD__
//#define __PS4_GAMEPAD__

//
#if defined(__GOBLE__) 
#undef __PS2_GAMEPAD__
#undef __PS4_GAMEPAD__
#elif defined(__PS2_GAMEPAD__) 
#undef __GOBLE__
#undef __PS4_GAMEPAD__
#elif defined(__PS4_GAMEPAD__) 
#undef __GOBLE__
#undef __PS2_GAMEPAD__
#endif

//
#ifdef __GOBLE__
//#define GOBLE_BAUD_RATE 38400
#define GOBLE_BAUD_RATE 115200
#endif // __GOBLE__

#ifdef __PS4_GAMEPAD__
#define PS4_MAC_ADDR "b0:e5:ed:65:26:c3"  // !! replace with your own DualShock4 Controller Bluetooth MAC address
#endif

//ps2 gamepad port definitions - SPI
#ifdef __PS2_GAMEPAD__
#define PS2_ATT 2   // cs
#define PS2_CMD 23  // mosi
#define PS2_DAT 19  // miso
#define PS2_CLK 18  // sck
#endif // __PS2_GAMEPAD__
#define BUZZER_PIN 4
#define SERVO_CAL_PIN 25
#define OE_PIN 26

#define PCA9685_I2C_ADDR  0x41
#endif //__CONFIG_
