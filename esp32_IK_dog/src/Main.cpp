/*
  Quadruped robot arduino sketch.
  This code is modification of https://github.com/maestrakos/quad by Alexandros Petkos
  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/
#include <Arduino.h>
#include "Config.hpp"
#include "Buzzer.hpp"
#include "Hardware.hpp"
#include "Kinematics.hpp"
/*
  ==============================
  HARDWARE - control method
  ==============================
*/
#ifdef __PS4_GAMEPAD__
#include <PS4Controller.h>
#endif
//
#ifdef __PS2_GAMEPAD__
#include "PS2X_lib.h" //reference: http://www.billporter.info/
const int RUMBLE = true;
const int PRESSURES = false;
// gamepad variables
int gamepad_error;
byte gamepad_type;
byte gamepad_vibrate;
PS2X ps2x;
#endif //__PS2_GAMEPAD__
//
#ifdef __GOBLE__
#include "GoBLE.hpp"
#endif //__GOBLE__

//
Buzzer buzzer(BUZZER_PIN);
Hardware hardware(PCA9685_I2C_ADDR);
Kinematics kinematics(hardware);
float vo = kinematics.vrt_offset, ho = kinematics.hrz_offset;

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0;   //> indicates the direction of rotation
float height = 0; //> indicates the leg extension

int state = 0;        //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
float _period = 10.0; //> indicates the number of steps every second

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body

int8_t joystickLX = 0;
int8_t joystickLY = 0;
int8_t joystickRX = 0;
int8_t joystickRY = 0;

unsigned long duration;
//
//int sample_sum, sample_num = 10,
//                sample_index;
//float freq;

//: handle input paramters
float stick_min = 6.f;
float lx, ly, rx, ry;



#ifdef __PS2_GAMEPAD__
void init_input_ps2()
{
  for (int i = 0; i < 3; i++)
  {
    gamepad_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);
    if (gamepad_error == 0)
    {
      Console.println("Found Controller, configured successful;\n");
      break;
    }
    delay(1000);
  }
  if (gamepad_error == 1)
  {
    Console.print("No PS2 controller found: ");
    Console.println(gamepad_error);
    aborted();
  }
  else if (gamepad_error == 2)
  {
    Console.print("PS2 Controller found but not accepting commands: ");
    Console.println(gamepad_error);
    aborted();
  }

  //verify the gamepad type
  gamepad_type = ps2x.readType();
  if (gamepad_type == 0)
    Console.println("Unknown PS2 Controller type found");
  else if (gamepad_type == 1)
    Console.println("DualShock Controller found");
  else if (gamepad_type == 2)
    Console.println("GuitarHero Controller found");
  else if (gamepad_type == 3)
    Console.println("Wireless Sony DualShock Controller found");

  //turn off gamepad vibration
  gamepad_vibrate = 0;
}
#endif //__PS2_GAMEPAD__

inline void init_input() {
#ifdef __PS2_GAMEPAD__
  init_input_ps2();
  Console.println("Using PS2 Controller!");
#elif defined(__GOBLE__)
  Goble.begin(GOBLE_BAUD_RATE);
  Console.println("Using BlueTooth!");
#elif defined(__PS4_GAMEPAD__)
  PS4.begin((char*)PS4_MAC_ADDR);
  Console.println("Using PS4 Controller!");
#else
  Console.println("No Controller!");
#endif
}

void aborted()
{
  Console.println("Program aborted!");
  buzzer.beepError();
  while (1)
    ;
}


void handle_input()
{
  lx = Kinematics::inter(lx, joystickLX / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick
  ly = Kinematics::inter(ly, joystickLY / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick
  rx = Kinematics::inter(rx, joystickRX / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick
  ry = Kinematics::inter(ry, joystickRY / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick
  //Console.println("joystickLY: " + String(joystickLY) + ", ly: " + String(ly));
  //Console.println("joystickRX: " + String(joystickRX) + ", rx: " + String(rx));
  //Console.println("joystickRY: " + String(joystickRY) + ", ry: " + String(ry));
  if (abs(lx) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float x0 = lx - stick_min * Kinematics::sign(lx); //> subtracts the deadzone
    if (state == 1)
    {
      _direction.y = 0; //x0 / 10.f;
    }
    else if (state != 4)
    {
      _direction.y = x0 / 2;
    }
  }
  else
    _direction.y = 0;

  if (abs(ly) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float y0 = ly - stick_min * Kinematics::sign(ly); //> subtracts the deadzone
    if (state == 1)
    {
      _direction.x = y0 / 10.f;
      if (y0 > 0)
        kinematics.vrt_offset = Kinematics::inter(kinematics.vrt_offset, vo - 6.f, 2.f);
      else
        kinematics.vrt_offset = Kinematics::inter(kinematics.vrt_offset, vo + 3.f, 2.f);
    }
    else if (state != 4)
    {
      _direction.x = y0 / 2;
      kinematics.vrt_offset = vo;
    }
  }
  else
  {
    _direction.x = 0;
    kinematics.vrt_offset = vo;
  };

  if (abs(rx) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float x1 = rx - stick_min * Kinematics::sign(rx); //> subtracts the deadzone
    if (state == 1)
      turn = x1 / 16.f;
    else if (state != 4)
      turn = x1;
  }
  else
    turn = 0;

  if (abs(ry) > stick_min)
  { //> checks whether the stick position is out of the deadzone
    float y1 = ry - stick_min * Kinematics::sign(ry); //> subtracts the deadzone
    height = y1;
  }
  else
    height = 0;

  //if (PS4.data.button.touchpad)  //> checks the touchpad state
  //#ifdef __PS2_GAMEPAD__
  //  if (ps2x.ButtonPressed(PSB_CIRCLE))
  //  {
  //    if (_tb == true)
  //    {
  //      _tb = false;
  //      state++;
  //      if (state > 4)
  //        state = 0;
  //      buzzer.beepShort();
  //      //Console.println("state: " + String(state));
  //    }
  //  }
  //  else
  //    _tb = true;
  //#endif
}

#ifdef __DEBUG__
#define properties 0
void commands_exe(float val1, float val2, float val3)
{
  //: propertios 0 is used to calibrate the joints
  if (properties == 0)
  {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Console.print("- leg ");
    Console.print(leg);
    Console.print(" joint ");
    Console.print(joint);
    Console.print(" set to ");
    Console.print(servo);
    Console.print(".\n");

    hardware.set_servo(leg, joint, servo);
  }
  //: propertios 1 is used for small adjustments to balance the weight
  else if (properties == 1)
  {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Console.print("- leg ");
    Console.print(leg);
    Console.print(" null ");
    Console.print(empty);
    Console.print(" set to ");
    Console.print(ammount);
    Console.print(".\n");

    kinematics.base_offset[leg] = ammount;
  }
}

// !! make sure you have enabled Newline or Carriage return
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input
void handle_serial()
{
  //: reads and stores the serial data
  int i = 0;
  float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Console.available())
  {
    char c = Console.read();
    if (c == 13 || c == 32 || c == '\n')
    {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    }
    else
      s_buff += c;
  }

  if (_mode == 0)
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)
    if (state == 4)
    {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}
#endif //__DEBUG__

void setup()
{
#ifdef __DEBUG__
  Console.begin(115200);
  Console.println("in debugging mode");
#endif
  hardware.init_hardware();
  init_input();
  //: servo calibration mode - while PIN 25 connects to 3.3V, all servos in 90° for servo arm adjustment °
  pinMode(SERVO_CAL_PIN, INPUT_PULLDOWN);
  while (digitalRead(SERVO_CAL_PIN)) {
    hardware.set_offset_with_command();
    delay(1000);
  }
  //
  buzzer.beepShort();
  Console.println("[started]");
}

void loop()
{
  //hardware.testGPIOservos(); return;
  //
  duration = millis();
  // this code gets the frequency of the loop function
  /*sample_sum += 1000.0 / (millis() - duration);
    sample_index++;

    if (sample_index > sample_num) {
    freq = sample_sum / sample_num;
    Console.println(freq);
    sample_sum = 0;
    sample_index = 0;
    }*/
  hardware.handle_hardware();
  kinematics.handle_kinematics(state, _direction, turn, height, _period);
  //
  //: test mode -  while PIN 25 connects to 3.3V again, will walk in trot gait
  static bool testMode = false;
  static long checkDuration = 0;
  if ((duration - checkDuration) > 1000) {
    checkDuration = duration;
    if (digitalRead(SERVO_CAL_PIN)) {
      hardware.attach();
      joystickLY = 127;
      joystickRX = 127;
      state = 1;
      testMode = true;
      goto __handle_input;
    } else if (testMode) {
      joystickLX = 0; joystickLY = 0;
      joystickRX = 0; joystickRY = 0;
      testMode = false;
      goto __handle_input;
    }
  }

#ifdef __GOBLE__
  static long previousDuration = 0;
  if ((duration - previousDuration) > 60000) {
    previousDuration = duration;
    hardware.detach(); // turn off servos while not moving for 1 min
    joystickLX = 0; joystickLY = 0;
    joystickRX = 0; joystickRY = 0;
    buzzer.beepShort();
    Console.println("stopped servos for power saving!");
    return;
  }
  if (Goble.available()) {
    previousDuration = duration;
    hardware.attach(); // turn on servos if they are off
    switch (state) {
      case 1:
        joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
        break;
      case 0:
      case 2:
        joystickRY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
        break;
      case 3:
        joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickLX = map(Goble.readJoystickX(), 0, 255, 127, -128);
        break;
      default:
        joystickLX = 0; joystickLY = 0;
        joystickRX = 0; joystickRY = 0;
    }

    if (Goble.readSwitchUp() == PRESSED) {
      state = 0;
      buzzer.beepShort();
    } else if (Goble.readSwitchDown() == PRESSED) {
      state = 2;
      buzzer.beepShort();
    } else if (Goble.readSwitchLeft() == PRESSED) {
      state = 3;
      buzzer.beepShort();
    } else if (Goble.readSwitchRight() == PRESSED) {
      state = 1;
      buzzer.beepShort();
    }

    if (Goble.readSwitchMid() == PRESSED) {
      buzzer.beepShort();
    } else if (Goble.readSwitchSelect() == PRESSED) {
      buzzer.beepShort();
    } else if (Goble.readSwitchAction() == PRESSED) {
      buzzer.beepShort();
    } else if (Goble.readSwitchStart() == PRESSED) {
      buzzer.beepShort();
      hardware.detach();
      joystickLX = 0; joystickLY = 0;
      joystickRX = 0; joystickRY = 0;
      previousDuration = duration;
    }
  }
#else // __GOBLE__
  static long previousDuration = 0;
  if ((duration - previousDuration) > 30) {
    previousDuration = duration;
#if defined(__PS4_GAMEPAD__)
    if (PS4.isConnected()) {
      joystickLX = PS4.data.analog.stick.lx;
      joystickLY = PS4.data.analog.stick.ly;
      joystickRX = PS4.data.analog.stick.rx;
      joystickRY = PS4.data.analog.stick.ry;
      if ( PS4.data.button.up ) {
        state = 0;
        buzzer.beepShort();
      } else if ( PS4.data.button.down ) {
        state = 2;
        buzzer.beepShort();
      } else if ( PS4.data.button.left ) {
        state = 3;
        buzzer.beepShort();
      } else if ( PS4.data.button.right ) {
        state = 1;
        buzzer.beepShort();
      } else  if ( PS4.data.button.triangle ) {
        hardware.detach();
        buzzer.beepShort();
      } else if ( PS4.data.button.cross ) {
        hardware.attach();
        buzzer.beepShort();
      }
    }
#elif defined(__PS2_GAMEPAD__)
    ps2x.read_gamepad(false, gamepad_vibrate);
    joystickLX = map(ps2x.Analog(PSS_LX), 0, 255, 127, -128);
    joystickLY = map(ps2x.Analog(PSS_LY), 0, 255, 127, -128);
    joystickRX = map(ps2x.Analog(PSS_RX), 255, 0, 127, -128);
    joystickRY = map(ps2x.Analog(PSS_RY), 255, 0, 127, -128);
    //        Console.println("joystickLY: " + String(joystickLY) + ", joystickLX: " + String(joystickLX));
    //        Console.println("joystickRY: " + String(joystickRY) + ", joystickRX: " + String(joystickRX));

    if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
      state = 2;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
      state = 3;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_PAD_UP)) {
      state = 0;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
      state = 1;
      buzzer.beepShort();
    } else  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      state = 4;
      buzzer.beepShort();
    }
#endif  // ps2/ps4_GAMEPAD__
  }
#endif  //__GOBLE__
  //
__handle_input:
  handle_input();
  //
#ifdef __DEBUG__
  if (Console.available())
    handle_serial();
#endif
}
