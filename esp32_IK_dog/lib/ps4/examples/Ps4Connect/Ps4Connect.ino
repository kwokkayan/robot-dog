#include <PS4Controller.h> //https://github.com/aed3/PS4-esp32

void setup()
{
    Serial.begin(115200);
    PS4.begin("0c:fc:83:e9:a0:5d");
    Serial.println("Ready.");
}

void loop()
{
  if (PS4.isConnected()){
    Serial.println("Connected!");
  }

  delay(3000);
}
