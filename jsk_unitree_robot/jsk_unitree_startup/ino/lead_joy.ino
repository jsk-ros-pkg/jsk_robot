/*
  Thumb Joystick demo v1.0
  by:https://www.seeedstudio.com
  connect the module to A0&A1 for using;
*/

#include <ros.h>
#include <sensor_msgs/Joy.h>
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
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_RGB + NEO_KHZ800);

int chkButton (byte pinBut, unsigned long ButTimeout);
enum { None, SingleClick, DoubleClick };

// ros
float DeadZone;


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
    // Pixels
    pixels.clear();
    pixels.setPixelColor(0, pixels.ColorHSV((counter*4000)%65535, 255, 64));
    pixels.show();
    
    // Joystick
    float a0 = (analogRead(A0)-512)/256.0;
    float a1 = (analogRead(A1)-512)/256.0;
    if ( (a0*a0 + a1*a1) < DeadZone*DeadZone ) {
      a0 = a1 = 0;
    }

    joy_msg.header.stamp = nh.now();
    joy_msg.axes[1] = a1;
    joy_msg.axes[0] = a0;
    
    //joy_msg.axes[2] = analogRead(A0);
    //joy_msg.axes[3] = analogRead(A1);
    //joy_msg.axes[4] = DeadZone;
    
    static int button_pressed = 0;
    if ( button_pressed == 0 ) {
      int b0 = chkButton(BUTTON_PIN, 250);
      if ( b0 == SingleClick ) {
        joy_msg.buttons[1] = 1;
        joy_msg.buttons[2] = 0;
        strip.setPixelColor(0, strip.Color(255,0,0));
        button_pressed = 10;
      } else if ( b0 == DoubleClick )  {
        joy_msg.buttons[1] = 0;
        joy_msg.buttons[2] = 1;
        strip.setPixelColor(0, strip.Color(0,0,255));
        button_pressed = 10;
      } else {
        joy_msg.buttons[1] = 0;
        joy_msg.buttons[2] = 0;
        strip.setPixelColor(0, strip.Color(0,255,0));
      }
    }
    if ( button_pressed > 0 ) {
      button_pressed = button_pressed - 1;
    }
    strip.show();

    // publish
    pub.publish(&joy_msg);
    nh.spinOnce();
    delay(100);

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
