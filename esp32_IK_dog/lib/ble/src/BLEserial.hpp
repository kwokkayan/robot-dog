#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "QueueArray.hpp"

#define SERVICE_UUID "DFB0"
#define CHARACTERISTIC_UUID "DFB1"

#define BUFF_LEN 32

class BLEserial: public BLECharacteristicCallbacks, BLEServerCallbacks {
  private:
    BLEServer *pServer;
    BLECharacteristic *pRxTxCharacteristic;
    volatile bool locked = false;
    //
    uint8_t outBuffer[BUFF_LEN];
    volatile size_t dataLen = 0;
    //
    QueueArray <byte> dataQueue;

    // Characteristic User Description
    class BLE2901 : public BLEDescriptor {
      public:
        BLE2901() : BLEDescriptor(BLEUUID((uint16_t)0x2901)) {
          std::string data = "TX & RX";
          setValue(data);
        }
    };

    // Client Characteristic Configuration
    class BLE2902 : public BLEDescriptor {
      public:
        BLE2902() : BLEDescriptor(BLEUUID((uint16_t)0x2902)) {
          uint8_t data[] = {0};
          setValue(data, 1);
        }
    };
    // PnP ID
    class BLE2A50 : public BLEDescriptor {
      public:
        BLE2A50() : BLEDescriptor(BLEUUID((uint16_t)0x2A50)) {
          uint8_t data[7] = {0x01, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x01};
          setValue(data, 7);
        }
    };

    //BLECharacteristicCallbacks
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        locked = true;
        //        Serial.println("Queue bytes1: " + String(dataQueue.count()));
        //        Serial.println("Received bytes: " + String(rxValue.length()));
        for (int i = 0; i < rxValue.length(); i++) {
          dataQueue.enqueue(rxValue[i]);
        }
        //        Serial.println("Queue bytes2: " + String(dataQueue.count()));
        //        Serial.println();
        locked = false;
      }
    }
    void onRead(BLECharacteristic *pCharacteristic) {
      if (dataLen > 0) {
        pCharacteristic->setValue(outBuffer, dataLen);
        dataLen = 0;
      }
    }

    void onNotify(BLECharacteristic *pCharacteristic) {

    }

    //BLEServerCallbacks
    void onConnect(BLEServer *pServer) {
      //Serial.println("connected!");
    };
    void onDisconnect(BLEServer *pServer) {
      pServer->startAdvertising();
      //Serial.println("start advertising again!");
    }

  public:
    BLEserial()
      : pServer(NULL) {}

    void begin(String id = "") {
      uint64_t chipid = ESP.getEfuseMac();
      if (id.length() == 0) {
        id = "BLE" + String((uint16_t)(chipid >> 32), HEX);
      }
      id.toUpperCase();
      // Create the BLE Device
      BLEDevice::init(id.c_str());
      //
      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(this);
      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID);
      // Create the Characteristic
      pRxTxCharacteristic = pService->createCharacteristic(
                              CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE |
                              BLECharacteristic::PROPERTY_WRITE_NR |
                              BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_NOTIFY);
      pRxTxCharacteristic->addDescriptor(new BLE2902());
      pRxTxCharacteristic->addDescriptor(new BLE2901());
      pRxTxCharacteristic->setCallbacks(this);
      // Start the service
      pService->start();

      /* Start advertising */
      // BLEAdvertising *pAdvertising = pServer->getAdvertising();
      // pAdvertising->addServiceUUID(SERVICE_UUID);
      // pServer->startAdvertising();
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(SERVICE_UUID);
      //
      // BLEAdvertisementData oAdvertisementData;
      // oAdvertisementData.setShortName("goble");
      // oAdvertisementData.setName("GoBLE");
      // oAdvertisementData.setManufacturerData("makerlab");
      // pAdvertising->setAdvertisementData(oAdvertisementData);
      pAdvertising->start();
      //Serial.println(id + ", waiting a client connection to notify...");
    }

    bool isConnected() {
      return pServer->getConnectedCount();
    }

    int read() {
      if (dataQueue.isEmpty() || !isConnected() || locked) {
        return -1;
      }
      return dataQueue.pop();
    }

    bool available() {
      if (dataQueue.isEmpty() || !isConnected() || locked) {
        return false;
      }
      return true;
    }

    size_t write(uint8_t *buff, size_t len) {
      if (len > BUFF_LEN) len = BUFF_LEN;
      memcpy(this->outBuffer, buff, len);
      this->dataLen = len;
      return len;
    }
};
