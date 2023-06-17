#ifndef __BUZZER__
#define __BUZZER__
class Buzzer {
    int pin;
  private:
    void tone(int pin, int frequency, int duration) {
      const short resolution = 8;
      const short channel = 15;
      const short soundOn =  (1 << (resolution - 1)); // 50% duty cycle
      const short soundOff = 0; // 0% duty cycle
      ledcSetup(channel, frequency, resolution);  // Set up PWM channel
      ledcAttachPin(pin, channel);                      // Attach channel to pin
      ledcWrite(channel, soundOn);
      delay(duration);
      ledcWrite(channel, soundOff);
      ledcDetachPin((gpio_num_t)pin);
    }

  public:
    Buzzer(int pin): pin(pin) {
    }
    void beep() {
      tone(pin, 500, 500);
    }
    void beepError() {
      tone(pin, 100, 1000);
    }
    void beepShort() {
      tone(pin, 50, 100);
    }
};
#endif //__BUZZER__
