#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__) || defined(__IMXRT1062__)
  #if defined(USE_TEENSY_HW_SERIAL)
    #define SERIAL_CLASS HardwareSerial // Teensy HW Serial
  #else
    #include <usb_serial.h>  // Teensy 3.0 and 3.1
    #define SERIAL_CLASS usb_serial_class
  #endif
#elif defined(_SAM3XA_)
  #include <UARTClass.h>  // Arduino Due
  #define SERIAL_CLASS UARTClass
#elif defined(USE_USBCON)
  // Arduino Leonardo USB Serial Port
  #define SERIAL_CLASS Serial_
#elif (defined(__STM32F1__) and !(defined(USE_STM32_HW_SERIAL))) or defined(SPARK) 
  // Stm32duino Maple mini USB Serial Port
  #define SERIAL_CLASS USBSerial
#elif defined(USE_BLUETOOTH)
  #include <BluetoothSerial.h>
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif

  #if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
  #endif
  static BluetoothSerial bluetoothSPP;
  #define SERIAL_CLASS BluetoothSerial
#else 
  #include <HardwareSerial.h>  // Arduino AVR
  #define SERIAL_CLASS HardwareSerial
#endif

class ArduinoHardware
{
public:
  ArduinoHardware(SERIAL_CLASS *io, long baud = 500000)
  {
    iostream = io;
    baud_ = baud;
  }
  ArduinoHardware()
  {
#if defined(USE_BLUETOOTH)
    iostream = &bluetoothSPP;
#else
    iostream = &Serial;
#endif
    baud_ = 500000;
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
#if defined (USE_BLUETOOTH)
    uint64_t chipid = ESP.getEfuseMac();
    String id = String((uint16_t)(chipid >> 32), HEX);
    id.toUpperCase();
    iostream->begin("KKY-" + id);
#else
#if defined(USE_USBCON)
      // Startup delay as a fail-safe to upload a new sketch
      delay(3000); 
#endif
      iostream->begin(baud_);
#endif
  }

  int read() { 
#if defined (USE_BLUETOOTH)
    if (iostream->connected()) {
      return iostream->read(); 
    } else {
      return -1;
    }
#else
    return iostream->read();
#endif
  };
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