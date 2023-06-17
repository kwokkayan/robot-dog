#ifndef GOBLE_H_
#define GOBLE_H_

#include "QueueArray.hpp"

//#define ESP32_SPP
#define ESP32_BLE

// defines Console and BlueTooth devices
#define Console Serial
#ifdef ESP32_SPP
#include <BluetoothSerial.h>
#elif defined(ESP32_BLE)
#include "BLEserial.hpp"
#endif


/**************** Debugger Configuration ******************/
//#define __DEBUG_PACKET__
#ifdef __DEBUG_PACKET__
#define DEBUGDATARECEIVER 1
#define DEBUGDATARAW      0
#define DEBUGPARSER       0
#define DEBUGCHECKSUM     1
#define DEBUGUPDATEBUTTON 1
#else
#define DEBUGDATARECEIVER 0
#define DEBUGDATARAW      0
#define DEBUGPARSER       0
#define DEBUGCHECKSUM     0
#define DEBUGUPDATEBUTTON 0
#endif

const byte SWITCH_1       = 1;
const byte SWITCH_2       = 2;
const byte SWITCH_3       = 3;
const byte SWITCH_4       = 4;
const byte SWITCH_5       = 5;
const byte SWITCH_6       = 6;
const byte SWITCH_7       = 7;
const byte SWITCH_8       = 8;
const byte SWITCH_9       = 9;
const byte SWITCH_10      = 10;
const byte SWITCH_11      = 11;
const byte SWITCH_12      = 12;

const byte SWITCH_UP      = SWITCH_1;
const byte SWITCH_RIGHT   = SWITCH_2;
const byte SWITCH_DOWN    = SWITCH_3;
const byte SWITCH_LEFT    = SWITCH_4;

const byte SWITCH_SELECT  = SWITCH_5;
const byte SWITCH_START   = SWITCH_6;
const byte SWITCH_ACTION  = SWITCH_7;
const byte SWITCH_MID     = SWITCH_8;

const byte SWITCH_TILT_UP = SWITCH_9;
const byte SWITCH_TILT_DN = SWITCH_10;
const byte SWITCH_PAN_LF  = SWITCH_11;
const byte SWITCH_PAN_RT  = SWITCH_12;

/*
   These constants can be use for comparison with the value returned
   by the readButton() method.
*/
const boolean PRESSED   = LOW;
const boolean RELEASED  = HIGH;

/*
   Data structure for the command buffer

*/

// Package protocol configuration
#define PACKHEADER          1
#define PACKHEADER2         2
#define PACKADDRESS         3
#define PACKBUTTONSIGN      4
#define PACKJOYSTICKSIGN    5
#define PACKPAYLOAD         6


#define DEFAULTHEADER1          0x55
#define DEFAULTHEADER2          0xAA
#define DEFAULTADDRESS          0x11
#define DEFAULTPACKLENGTH 10

#define MAXBUTTONID             SWITCH_12

#define PARSESUCCESS            0x10

//DL package
#pragma pack(1)
typedef struct
{
  byte  header1;          // 0x55
  byte  header2;          // 0xAA
  byte  address;          // 0x11

  byte  latestDigitalButtonNumber;
  byte  digitalButtonNumber;

  byte  joystickPosition;
  byte  buttonPayload[MAXBUTTONID];
  byte  joystickPayload[4];
  byte  checkSum;

  byte  commandLength;
  byte  parseState;
  boolean commandFlag;
} sDataLink;
#pragma pack()

class _GoBLE {

  public:
    _GoBLE();

    void begin(unsigned long baudrate = 115200);
    boolean available();

    int readJoystickX();
    int readJoystickY();
    int readJoystickX2();
    int readJoystickY2();
    /*
       Reads the current state of a button. It will return
       LOW if the button is pressed, and HIGH otherwise.
    */
    boolean readSwitchUp();
    boolean readSwitchDown();
    boolean readSwitchLeft();
    boolean readSwitchRight();
    boolean readSwitchSelect();
    boolean readSwitchStart();
    boolean readSwitchAction();
    boolean readSwitchMid();
    boolean readSwitchPanLf();
    boolean readSwitchPanRt();
    boolean readSwitchTiltUp();
    boolean readSwitchTiltDn();

  private:
    sDataLink rDataPack;
    // create a queue of characters.
    QueueArray <byte> bleQueue;

    byte _joystickX, _joystickY;
    byte _joystickX2, _joystickY2;
    byte _button[MAXBUTTONID + 1]; // index 0 is not used, thus plus 1

    void updateJoystickVal();
    void updateButtonState();

    void initRecvDataPack();
    int bleDataPackageParser();
    void bleDataReceiver();
    
  public:
#ifdef ESP32_SPP
    BluetoothSerial BlueTooth;
#elif defined(ESP32_BLE)
    BLEserial BlueTooth;
#else
    HardwareSerial &BlueTooth;
#endif
};

extern _GoBLE Goble;

#endif // GOBLE_H_
