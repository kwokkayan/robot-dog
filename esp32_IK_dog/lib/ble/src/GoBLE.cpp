#include "GoBLE.hpp"

_GoBLE Goble;

#if !defined(ESP32_SPP) && !defined(ESP32_BLE)
_GoBLE::_GoBLE():BlueTooth(Serial2) {
#else
_GoBLE::_GoBLE() {
#endif  
  initRecvDataPack();

  _joystickX = 127;
  _joystickY = 127;
  _joystickX2 = 127;
  _joystickY2 = 127;
  for (int i = 0; i < MAXBUTTONID + 1; i++) {
    _button[i] = RELEASED;
  }

  for (int i = 0; i < 20; i++) bleQueue.push(0x00);
  for (int i = 0; i < 20; i++) bleQueue.pop();
}

void _GoBLE::begin(unsigned long baudrate) {
#ifdef ESP32_SPP
  uint64_t chipid = ESP.getEfuseMac();
  Console.printf("ESP32 BlueTooth SPP %04X",
                 (uint16_t)(chipid >> 32));    // print High 2 bytes
  Console.printf("%08X\n", (uint32_t)chipid);  // print Low 4bytes.
  // String id = "mobile" + String((uint16_t)(chipid >> 32), HEX);
  String id = "WM-" + String((uint32_t)chipid, HEX);
  id.toUpperCase();
  BlueTooth.begin(id);
  Console.println("ESP32 BlueTooth SPP - " + id);
#elif defined(ESP32_BLE)
  BlueTooth.begin();
#else 
  BlueTooth.begin(baudrate);
#endif
}

boolean _GoBLE::available() {
  /*
    function introduction:
     push the new valid data to the data buffer package
     throw away the invalid byte
     parse the data package when the command length is matching the protocol
  */

  if (BlueTooth.available()) bleDataReceiver();
  /*
    if (DEBUGDATARAW) {
      Console.println("GoBLE availalbe -> new data package!");
      for (int i = 0; i < rDataPack.commandLength; i++) {
        Console.print(bleQueue.pop(), HEX);
      }
      Console.println();
    }

    if (DEBUGPARSER) {
      Console.print("GoBLE availalbe -> bleQueue Counter: ");
      Console.print(bleQueue.count());
      Console.println();
    }
  */
  if (rDataPack.commandFlag && bleQueue.count() == rDataPack.commandLength) {
    rDataPack.parseState = bleDataPackageParser();

    if (rDataPack.parseState == PARSESUCCESS) {
      updateJoystickVal();
      updateButtonState();
      return true;
    }
  }

  return false;
}

int _GoBLE::readJoystickX() { return _joystickX; }
int _GoBLE::readJoystickY() { return _joystickY; }
int _GoBLE::readJoystickX2() {
  if (rDataPack.address == 0x11) return 127;
  return _joystickX2;
}
int _GoBLE::readJoystickY2() {
  if (rDataPack.address == 0x11) return 127;
  return _joystickY2;
}
boolean _GoBLE::readSwitchUp() { return _button[SWITCH_UP]; }

boolean _GoBLE::readSwitchDown() { return _button[SWITCH_DOWN]; }

boolean _GoBLE::readSwitchLeft() { return _button[SWITCH_LEFT]; }

boolean _GoBLE::readSwitchRight() { return _button[SWITCH_RIGHT]; }

boolean _GoBLE::readSwitchSelect() { return _button[SWITCH_SELECT]; }

boolean _GoBLE::readSwitchStart() { return _button[SWITCH_START]; }

boolean _GoBLE::readSwitchAction() { return _button[SWITCH_ACTION]; }

boolean _GoBLE::readSwitchMid() { return _button[SWITCH_MID]; }

boolean _GoBLE::readSwitchPanLf() { return _button[SWITCH_PAN_LF]; }

boolean _GoBLE::readSwitchPanRt() { return _button[SWITCH_PAN_RT]; }

boolean _GoBLE::readSwitchTiltUp() { return _button[SWITCH_TILT_UP]; }

boolean _GoBLE::readSwitchTiltDn() { return _button[SWITCH_TILT_DN]; }

// Private functions

int _GoBLE::bleDataPackageParser() {
  /*
    0x10  - Parse success
    0x11  - Wrong header charactors
    0x12  - Wrong button number
    0x13  - Check Sum Error
  */
  byte calculateSum = 0;

  rDataPack.header1 = bleQueue.pop(), calculateSum += rDataPack.header1;
  rDataPack.header2 = bleQueue.pop(), calculateSum += rDataPack.header2;

  if (rDataPack.header1 != DEFAULTHEADER1) return 0x11;
  if (rDataPack.header2 != DEFAULTHEADER2) return 0x11;

  rDataPack.address = bleQueue.pop(), calculateSum += rDataPack.address;

  rDataPack.latestDigitalButtonNumber = rDataPack.digitalButtonNumber;
  rDataPack.digitalButtonNumber = bleQueue.pop(),
  calculateSum += rDataPack.digitalButtonNumber;
  if (0) {
    Console.print("digitalButtonNumber: ");
    Console.println(rDataPack.digitalButtonNumber);
  }

  int digitalButtonLength = rDataPack.digitalButtonNumber;

  if (DEBUGCHECKSUM) {
    Console.print("Parser -> digitalButtonLength: ");
    Console.println(digitalButtonLength);
  }
  if (digitalButtonLength > MAXBUTTONID) return 0x12;

  rDataPack.joystickPosition = bleQueue.pop(),
  calculateSum += rDataPack.joystickPosition;
  if (0) {
    Console.print("joystickPosition: ");
    Console.println(rDataPack.joystickPosition);
  }

  // read button data package - dynamic button payload length
  for (int buttonPayloadPointer = 0; buttonPayloadPointer < digitalButtonLength;
       buttonPayloadPointer++) {
    rDataPack.buttonPayload[buttonPayloadPointer] = bleQueue.pop();
    calculateSum += rDataPack.buttonPayload[buttonPayloadPointer];
  }
  // read 4 byte joystick data package
  for (int i = 0; i < 4; i++) {
    if (0) {
      Console.print("joystickPayload: ");
      Console.print(i);
      Console.print(" ");
      Console.println(rDataPack.joystickPayload[i]);
    }
    rDataPack.joystickPayload[i] = bleQueue.pop(),
    calculateSum += rDataPack.joystickPayload[i];
  }

  rDataPack.checkSum = bleQueue.pop();

  if (DEBUGCHECKSUM) {
    Console.print("Parser -> sum calculation: ");
    Console.println(calculateSum);

    Console.print("Parser -> checkSum byte value: ");
    Console.println(rDataPack.checkSum);
  }

  // check sum and update the parse state value
  // if the checksum byte is not correct, return 0x12

  rDataPack.commandFlag = false;

  if (rDataPack.checkSum == calculateSum)
    return PARSESUCCESS;
  else
    return 0x13;
}

void _GoBLE::bleDataReceiver() {
  byte inputByte = BlueTooth.read();

  if (DEBUGDATARECEIVER) {
    Console.print("bleDataReceiver() -> new data:");
    Console.print(inputByte, HEX);
    Console.print(", ");
    Console.println((int)inputByte, DEC);
  }

  // throw the trash data and restore the useful data to the queue buffer
  if (inputByte == DEFAULTHEADER1 || rDataPack.commandFlag == true) {
    bleQueue.push(inputByte);
    rDataPack.commandFlag = true;

    // auto adjust the command length based on the button command value
    if (bleQueue.count() == PACKBUTTONSIGN) {
      if (inputByte > 0 && inputByte < MAXBUTTONID + 1) {
        // default command length + button number
        rDataPack.commandLength = DEFAULTPACKLENGTH + inputByte;
        if (DEBUGDATARECEIVER) {
          Console.print("bleDataReceiver -> Command Length:");
          Console.println(rDataPack.commandLength);
        }
      } else
        rDataPack.commandLength = DEFAULTPACKLENGTH;
    }
  }
}

void _GoBLE::initRecvDataPack() {
  rDataPack.commandFlag = false;
  rDataPack.commandLength = DEFAULTPACKLENGTH;
  rDataPack.parseState = PARSESUCCESS;

  rDataPack.digitalButtonNumber = 0;
  rDataPack.latestDigitalButtonNumber = 0;
}

void _GoBLE::updateJoystickVal() {
  _joystickY = rDataPack.joystickPayload[0];
  _joystickX = rDataPack.joystickPayload[1];
  _joystickY2 = rDataPack.joystickPayload[2];
  _joystickX2 = rDataPack.joystickPayload[3];
}

void _GoBLE::updateButtonState() {
  if (rDataPack.digitalButtonNumber == 0 &&
      rDataPack.latestDigitalButtonNumber != 0) {
    for (int i = 0; i <= MAXBUTTONID; i++) {
      if (_button[i] == PRESSED) {
        if (DEBUGUPDATEBUTTON) {
          Console.print("updateButtonState -> clear Pressed button number: ");
          Console.println(i);
        }
        _button[i] = RELEASED;
      }
    }
  }

  for (int i = 0; i < rDataPack.digitalButtonNumber; i++) {
    _button[rDataPack.buttonPayload[i]] = PRESSED;
  }
}
