// 7seg GPS Clock by copyright 2018 by 'hibble'
//
// Hardware:
// -Arduino nano v3 
// -LED strip WS2811B etc... (5v)
// -logic Level shifter 3.3v/5v
// -Neo-6m GPS modual (3.3v max)
// -extra external GPS antena (if poor indoor signal but we only need time not a full GPS lock) 
//
//
//changelog summary (last 3 chanes for older see github https://github.com/hibble/7seg_clock_GPS_WS2811B
//v0.1 - initial version pre alpha code.

//--------------------------------
//Imports and declarations
//--------------------------------

#include <Adafruit_NeoPixel.h>
#include <TaskScheduler.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>


static NMEAGPS  gps;
static gps_fix  fix;

#define NUMPIXELS 35 // number of LEDs in a strip forming a 7seg dispaly
#define DIGITPIN1 4 //digit 0 hour1 position
#define DIGITPIN2 5 //digit 1 hour2 position
#define DIGITPIN3 6 //digit 2 minuet1 position
#define DIGITPIN4 7 //digit 3 minuet2 position

Adafruit_NeoPixel strip[] = { //here is the variable for the multiple strips forming the clock display //may need 5th for center dots
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN1, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN3, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN4, NEO_GRB + NEO_KHZ800)
};
const int bright = 40; //brightness for all pixels 0-255 range, 32 being dim

//--------------------------------
//Tasks scedualing
//--------------------------------
// Task timer settings
const int GPSMillis = 2000; //get GPS time every 2 sec (set to higher value once code is more meture e.g 1x evey 10min?)
const int DisplayMillis = 100; // update display every 10th of a second
const int SerialDisplayMillis = 1000; // serial output update 1s intervals

// task prototypes
void getGPSTime();
void update7segDisplay();
void updateSerialMonitor();

Scheduler runner;
Task t1 (GPSMillis, TASK_FOREVER, &getGPSTime);
Task t2 (DisplayMillis, TASK_FOREVER, &update7segDisplay);
Task t3 (SerialDisplayMillis, TASK_FOREVER, &updateSerialMonitor);

//--------------------------------
//Setup
//--------------------------------
void setup() {
  // put your setup code here, to run once:

}

//--------------------------------
//Main Loop
//--------------------------------
void loop() {
  // put your main code here, to run repeatedly:

}
