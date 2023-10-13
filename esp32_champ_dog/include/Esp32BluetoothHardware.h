#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif

#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

static BluetoothSerial bluetoothSPP;
#define SERIAL_CLASS BluetoothSerial

class ArduinoHardware
{
public:
  ArduinoHardware(SERIAL_CLASS *io, long baud = 57600)
  {
    iostream = io;
    baud_ = baud;
  }
  ArduinoHardware()
  {
    iostream = &bluetoothSPP;
    baud_ = 38400;
  }
  ArduinoHardware(ArduinoHardware &h)
  {
    this->iostream = h.iostream;
    this->baud_ = h.baud_;
  }

  void setPort(SERIAL_CLASS *io)
  {
    this->iostream = io;
  }

  void setBaud(long baud)
  {
    this->baud_ = baud;
  }

  int getBaud() { return baud_; }

  void init()
  {
    uint64_t chipid = ESP.getEfuseMac();
    String id = String((uint16_t)(chipid >> 32), HEX);
    id.toUpperCase();
    iostream->begin("KKY-" + id, false);
    Serial.println(String("Node handle on BT-SPP, address ") +
                   iostream->getBtAddressString());
  }

  int read() { return iostream->read(); };
  void write(uint8_t *data, int length)
  {
    iostream->write(data, length);
  }

  unsigned long time() { return millis(); }

protected:
  SERIAL_CLASS *iostream;
  long baud_;
};

#endif