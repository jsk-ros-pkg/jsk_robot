/*
  Thumb Joystick demo v1.0
  by:https://www.seeedstudio.com
  connect the module to A0&A1 for using;
*/

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
ros::NodeHandle nh;
sensor_msgs::Joy joy_msg;
ros::Publisher pub("joy", &joy_msg);

#include <Adafruit_NeoPixel.h> // requires veresion 1.10.2+
int Power = 11;
int PIN = 12;
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Mech Keycap
#define BUTTON_PIN D5
#define PIXEL_PIN D4
#define PIXEL_COUNT 60
#define PRESSED 1
#define UNPRESSED 0

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_RGB + NEO_KHZ800);

int chkButton (byte pinBut, unsigned long ButTimeout);
enum { None, SingleClick, DoubleClick, TripleClick,
       Quadruple, Quintuple, Sextuple, Septuple,
       Octuple, Nonuple, Decuple, Undecuple, Duodecuple};

// ros
float DeadZone;
int pressInterval;
int longPressInterval;
int debouncePeriod;
unsigned long pressCount = 0;
uint32_t currentColor;


uint32_t flatColors[] = {
  strip.Color(0, 255, 0), // 0: red
  strip.Color(0, 0, 255), // 1: blue
  strip.Color(255, 0, 0), // 2: green
  strip.Color(255, 255, 0), // 3: yellow
  strip.Color(255, 255, 255), // 4: white
  strip.Color(0, 178, 144), // 5: purple
  strip.Color(0, 128, 0),
  strip.Color(128, 128, 0),
  strip.Color(0, 128, 128),
  strip.Color(128, 0, 128),
  strip.Color(128, 128, 128),
  strip.Color(128, 64, 0),
  strip.Color(128, 192, 0),
};


void setPressedButton(long int* buttons, int button_length, int pressed_button) {
  for (size_t i = 0; i < button_length; ++i) {
    buttons[i] = 0;
  }
  if (0 <= pressed_button && pressed_button < button_length) {
    buttons[pressed_button] = 1;
  }
}

void setup()
{
    // ROS
    nh.initNode();
    nh.advertise(pub);
    joy_msg.axes = (float*)malloc(sizeof(float) * 6);
    joy_msg.axes_length = 6;
    for(int i = 0; i < joy_msg.axes_length; i++) joy_msg.axes[i] = 0;
    joy_msg.buttons = (long int*)malloc(sizeof(long int) * 12);
    joy_msg.buttons_length = 12;
    for(int i = 0; i < joy_msg.buttons_length; i++) joy_msg.buttons[i] = 0;

    while (!nh.connected()) {
      nh.spinOnce();
    }
    if ( ! nh.getParam("~deadzone", &DeadZone) ) {
      DeadZone = 0.1;
    }
    if ( ! nh.getParam("~press_interval", &pressInterval) ) {
      pressInterval = 500;
    }
    if ( ! nh.getParam("~debounce_period", &debouncePeriod) ) {
      debouncePeriod = 50;
    }

    currentColor = strip.Color(255, 0, 0);

    // LED
    pixels.begin();
    pinMode(Power, OUTPUT);
    digitalWrite(Power, HIGH);

    // Mech Keycap
    strip.begin();
    strip.clear();
    strip.show();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

int b0;
int counter = 0;
void loop()
{
    static unsigned long last_valid_input = millis();
    
    // Pixels
    pixels.clear();
    pixels.setPixelColor(0, pixels.ColorHSV((counter*4000)%65535, 255, 64));
    pixels.show();
    
    // Joystick
    float a0 = (analogRead(A0)-512)/256.0;
    float a1 = (analogRead(A1)-512)/256.0;
    if ( (a0*a0 + a1*a1) < DeadZone*DeadZone ) {
      a0 = a1 = 0;
    } else {
      last_valid_input = millis();
    }

    joy_msg.header.stamp = nh.now();
    joy_msg.axes[1] = a1;
    joy_msg.axes[0] = a0;

    //joy_msg.axes[2] = analogRead(A0);
    //joy_msg.axes[3] = analogRead(A1);
    //joy_msg.axes[4] = DeadZone;

    static int button_pressed = 0;
    if ( button_pressed == 0 ) {
      int b0 = pressed(BUTTON_PIN);
      button_pressed = 100;
      if (b0 >= 1) {
        std::string tmp = "pressed count: " + std::to_string(b0);
        nh.loginfo(tmp.c_str());
      }
      if (1 <= b0 && b0 < joy_msg.buttons_length) {
        setPressedButton(joy_msg.buttons, joy_msg.buttons_length, b0);
        currentColor = flatColors[b0];
      } else {
        setPressedButton(joy_msg.buttons, joy_msg.buttons_length, -1);
        currentColor = flatColors[0];  // red
        strip.setPixelColor(0, currentColor);
        button_pressed = 0;
      }
    }
    if ( button_pressed > 0 ) {
      button_pressed = button_pressed - 1;
      strip.setPixelColor(0, currentColor);
      last_valid_input = millis();
    }
    strip.show();

    // publish
    if ( (millis() - last_valid_input) < 1000) {
      pub.publish(&joy_msg);
    }
    nh.spinOnce();
    delay(10);

    counter = counter + 1;
}


// https://forum.arduino.cc/t/code-to-detect-single-or-double-click/915386/1

byte butLst;

// -----------------------------------------------------------------------------
int chkButton (byte pinBut, unsigned long ButTimeout)
{
    static unsigned long msecLst;
           unsigned long msec = millis ();

    if (msecLst && (msec - msecLst) > ButTimeout)  {
        msecLst = 0;
        return SingleClick;
    }

    byte but = digitalRead (pinBut);
    if (butLst != but)  {
        butLst = but;
        delay (10);           // **** debounce

        if (LOW == but)  {   // press
            if (msecLst)  { // 2nd press
                msecLst = 0;
                return DoubleClick;
            }
            else
                msecLst = 0 == msec ? 1 : msec;
        }
    }

    return None;
}

// -----------------------------------------------------------------------------


// https://forum.arduino.cc/t/one-button-couble-click-triple-click/509091/2

int pressed(byte pinBut)
{
  static uint32_t lastMillis = 0;
  static byte lastState = UNPRESSED;

  byte nowState = digitalRead(pinBut);
  /* if (nowState == PRESSED) { */
  /*   nh.loginfo("PRESSED"); */
  /* } else if (nowState == UNPRESSED) { */
  /*   nh.loginfo("UNPRESSED"); */
  /* } */

  if(nowState != lastState)
  {
    if(millis() - lastMillis < debouncePeriod) {
      return 0;
    }
    if (nowState == UNPRESSED) {
      lastMillis = millis();
      pressCount++;
      /* std::string tmp = "pressed cound: " + std::to_string(pressCount); */
      /* nh.loginfo(tmp.c_str()); */
    }
  }
  if (pressCount != 0) {
    if (millis() - lastMillis > pressInterval) {
      int presses = pressCount;
      pressCount = 0;
      return presses;
    }
  }
  lastState = nowState;
  return 0;
}
