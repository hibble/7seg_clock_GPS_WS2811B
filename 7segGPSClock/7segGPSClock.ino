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
//v1.5 - Fully working GPS clock - 3 colour mode.
//v1.0 - very basic and needs tidying but working.
//v0.1 - initial version pre alpha code.

//Future plans
//  .dfx files for laser cutting the clock
//  support auto summer time
//  support night node e.g auto turn off display when no one is in building.
//

//--------------------------------
//Imports and declarations
//--------------------------------
#include <Adafruit_NeoPixel.h>
#include <TaskScheduler.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

//--------------------------------
//User changable settings - note: UK colour used througout code not US spelling
//--------------------------------
//  write to NeoPixel seven segments, colours:
//  0 = off
//  1 = red
//  2 = green
//  3 = blue
//  4 = white
//  5 = yellow
//  6 = Random

//settings ibn following block can be changes depending on user need e.g colour and brightness
//  default is:
//  red = no gps/time to show/starting up

const int ledbrightness = 60; //  default 20 - brightness for all pixels 0-255 range.
int fullGPSFixColour = 2;     //  Green = Full GPS Sync with multiple satlights.
int partGPSFixColour = 5;     //  Yellow = time only/gps signal lost or not enouth satalights for full fix.
int noGPSFixColour = 1;       //  red = starting up/no signal/no time to show.
int centerDotsColour = 3;      //  blue
//End of user settings
//--------------------------------


static NMEAGPS  gps;
static gps_fix  fix;
int hours = 0;
int mins = 0;
boolean TickTock = false; //used to pulse center dots evey time display is updated.

#define NUMPIXELS 140 // number of LEDs in a strip forming a 7seg dispaly
#define DIGITPIN1 4 //digit 0 hour1 position
#define DIGITPIN2 5 //digit 1 hour2 position
#define DIGITPIN3 6 //digit 2 minuet1 position
#define DIGITPIN4 7 //digit 3 minuet2 position
#define DIGITPIN5 8 //digit 4 - 2 center dots

//#define EU_DST (bellow could be in if statments if multiple timezone support is needed)
static const uint8_t springMonth =  3;
static const uint8_t springDate  = 31; // latest last Sunday
static const uint8_t springHour  =  1;
static const uint8_t fallMonth   = 10;
static const uint8_t fallDate    = 31; // latest last Sunday
static const uint8_t fallHour    =  1;

// Set these values to the offset of your timezone from GMT
#define CALCULATE_DST
static const int32_t          zone_hours   = 0L; // GMT(London)
static const int32_t          zone_minutes = 0L; // usually zero
static const NeoGPS::clock_t  zone_offset  =
                                zone_hours   * NeoGPS::SECONDS_PER_HOUR +
                                zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

Adafruit_NeoPixel strip[] = { //here is the variable for the multiple strips forming the clock display //may need 5th for center dots
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN1, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN3, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(NUMPIXELS, DIGITPIN4, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4, DIGITPIN5, NEO_GRB + NEO_KHZ800)
};


//--------------------------------
//Tasks scedualing
//--------------------------------
//Task timer settings
const int DisplayMillis = 500; // update display every 1/2 of a second
const int SerialDisplayMillis = 2000; // serial output update 2s intervals

// task prototypes
void getGPSTime();
void update7segDisplay();
void updateSerialMonitor();

Scheduler runner;

Task t1 (DisplayMillis, TASK_FOREVER, &update7segDisplay);
Task t2 (SerialDisplayMillis, TASK_FOREVER, &updateSerialMonitor);

//--------------------------------
//Setup
//--------------------------------
void setup() {
  delay(2000); //needed for some arduinos serial to initilise,exrea 2 second startup time not a issue for a clock
  // setup perodic tasks
  runner.init();
  runner.addTask(t1);
  runner.addTask(t2);
  t1.enable();
  t2.enable();

  //setup serial
  DEBUG_PORT.begin(9600); //gps modual neo-6m is 9600 other gps moduals and difrent modes may need this adjusted.
  while (!DEBUG_PORT);

  DEBUG_PORT.print( F("NMEA.INO: started\n") );
  DEBUG_PORT.print( F("  fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("  gps object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

#ifndef NMEAGPS_RECOGNIZE_ALL
#error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

  DEBUG_PORT.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

#else
  if (gps.merging == NMEAGPS::NO_MERGING) {
    DEBUG_PORT.print  ( F("\nWARNING: displaying data from ") );
    DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
    DEBUG_PORT.print  ( F(" sentences ONLY, and only if ") );
    DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
    DEBUG_PORT.println( F(" is enabled.\n"
                          "  Other sentences may be parsed, but their data will not be displayed.") );
  }
#endif

  DEBUG_PORT.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  DEBUG_PORT.println( F(" sentence is received.\n"
                        "  You should confirm this with NMEAorder.ino\n") );

  trace_header( DEBUG_PORT );
  DEBUG_PORT.flush();
  //initialize serial for debugging and monotoring
  gpsPort.begin( 9600 ); //neo-6m works on 9600. difrent gps may need other speeds

  //LEDStrip array setup
  for (int s = 0; s < 5; s++) { //s is number of led strip digits in use and center dots.
    strip[s].begin(); // Initialize pins for output
    strip[s].setBrightness(ledbrightness); //brightness 0-255
    strip[s].show();  // Turn all LEDs off
    delay(200);
  }
  //flash dashes till we have gps time
  for (int t = 0; t < 4; t++) { // t is number of led strip digits in use.
    digitWrite(t, 8, 0); //blank
    strip[t].show();
    segLight(t, 7, noGPSFixColour); //start red untill gps time is avaliable
    strip[t].show();
    delay(400); //causes slight animation
  }
  // bit of time so red dashes show for minimum time.
  delay(1000);
}
//END void setup() stage
//////////////////////////////////

//--------------------------------
//Main Loop
//--------------------------------
void loop() {
  //main GPS code - was unhappy running as a periodic task
  while (gps.available( gpsPort )) {
    fix = gps.read();
    //if we have A valid GPS time and Date Adjust for GMT offset and DST
    if (fix.valid.time && fix.valid.date) {
      adjustTime( fix.dateTime );
      //for debuging could be removed
      DEBUG_PORT << fix.dateTime;
    }
    DEBUG_PORT.println();
  }
  //run our update tasks
  runner.execute();
}
//END void loop()
//////////////////////////////////


// Actually update the led strip 7 seg display.
// task called periodically by TaskScheduler
void update7segDisplay() {
  if (fix.status == 3) { // check if we have a full GPS fix
    hours = fix.dateTime.hours;
    mins = fix.dateTime.minutes;
    int hoursTens = hours / 10; //get the tens place of the hour
    int hoursOnes = hours % 10; //get the ones place of the hour
    int minsTens = mins / 10; //get the tens place of minutes
    int minsOnes = mins % 10; //get the ones place of minutes

    //colour set at start of file under usser settings.
    digitWrite(0, minsOnes, fullGPSFixColour);
    strip[0].show();
    digitWrite(1, minsTens, fullGPSFixColour);
    strip[1].show();
    digitWrite(2, hoursOnes, fullGPSFixColour);
    strip[2].show();
    digitWrite(3, hoursTens, fullGPSFixColour);
    strip[3].show();
    updateTickTock(); // update center dots
    //debug
    //trace_all( DEBUG_PORT, gps, fix );
  }
  else if (fix.status == 0) // not a full gps fix but some data is avaliable
    if (fix.valid.time) { //check if we have a GPS time if we do display it in yellow
      hours = fix.dateTime.hours;
      mins = fix.dateTime.minutes;
      int hoursTens = hours / 10; //get the tens place of the hour
      int hoursOnes = hours % 10; //get the ones place of the hour
      int minsTens = mins / 10; //get the tens place of minutes
      int minsOnes = mins % 10; //get the ones place of minutes

      //colour set at start of file under usser settings.
      digitWrite(0, minsOnes, partGPSFixColour);
      strip[0].show();
      digitWrite(1, minsTens, partGPSFixColour);
      strip[1].show();
      digitWrite(2, hoursOnes, partGPSFixColour);
      strip[2].show();
      digitWrite(3, hoursTens, partGPSFixColour);
      strip[3].show();
      updateTickTock(); // update center dots
      //debug
      //trace_all( DEBUG_PORT, gps, fix );
    }
    else { //no gps data avaliable
      //show dashes as red till we have gps time
      for (int t = 0; t < 4; t++) { // t is number ofled strip digits in use.
        digitWrite(t, 8, 0); //blank
        strip[t].show();
        segLight(t, 7, noGPSFixColour);//normal user colour overiden.
        strip[t].show();
      }
      //debug
      //Serial.println("Time not ready");
      //trace_all( DEBUG_PORT, gps, fix );
    }


}
//END void updateDisplay()
//////////////////////////

void updateTickTock() {
  //Serial.println("Time tock");
  if (TickTock == true) { //dots on
    digitWrite(4, 11, centerDotsColour); // 11 is patern for the dots
    strip[4].show();
    Serial.println("Time tockon");
  } else { //dots off
    digitWrite(4, 0, centerDotsColour); // 0 is used as that patern turns off led 0and1 other leds used for 0 do not exist on this strip.
    strip[4].show();
    Serial.println("Time tock off");
  }
  TickTock = !TickTock; // revers boolian state for next run.
}

void updateSerialMonitor() {
  //some code here
  trace_all( DEBUG_PORT, gps, fix );
}
//END void loop()
/////////////////

// shamless copy of other code see github for ninjaTimer
////////////////////////////////////////////////////////////////////////////////
void digitWrite(int digit, int val, int col) {

  //use this to light up a digit
  //'digit' is which one (right to left, 0 indexed)
  //'val' is the value to set on the digit
  //'col' is the color to use, R,G,B or W
  //example:
  //        digitWrite(0, 4, 2);
  //this would set the digit
  //on the far right to a "4" in green.

  /*
    // Letters are the standard naming, numbers are based upon the wiring sequence

            A 2
       ----------
      |          |
      |          |
    F 1 |          | B 3
      |          |
      |     G 7  |
       ----------
      |          |
      |          |
    E 6 |          | C 4
      |          |
      |     D 5  |
       ----------    dp 8

  */
  //these are the numeric character definitions,
  //if last argument is a 0, the segment is off
  //segLight 8 is the 2 dots but is shared with others being the first 2 leds 0-1
  if (val == 0) { // "0"
    //segments A,B,C,D,E,F
    segLight(digit, 1, col);
    segLight(digit, 2, col);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, col);
    segLight(digit, 6, col);
    segLight(digit, 7, 0);
    segLight(digit, 8, 0);
  }
  if (val == 1) { // "1"
    //segments A,B,C,D,E,F
    segLight(digit, 1, 0);
    segLight(digit, 2, 0);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, 0);
    segLight(digit, 6, 0);
    segLight(digit, 7, 0);
    segLight(digit, 8, 0);
  }
  if (val == 2) { // "2"
    //segments A,B,C,D,E,F
    segLight(digit, 1, 0);
    segLight(digit, 2, col);
    segLight(digit, 3, col);
    segLight(digit, 4, 0);
    segLight(digit, 5, col);
    segLight(digit, 6, col);
    segLight(digit, 7, col);
    segLight(digit, 8, col);
  }
  if (val == 3) { // "3"
    //segments A,B,C,D,E,F
    segLight(digit, 1, 0);
    segLight(digit, 2, col);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, col);
    segLight(digit, 6, 0);
    segLight(digit, 7, col);
    segLight(digit, 8, col);
  }
  if (val == 4) { // "4"
    //segments A,B,C,D,E,F
    segLight(digit, 1, col);
    segLight(digit, 2, 0);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, 0);
    segLight(digit, 6, 0);
    segLight(digit, 7, col);
    segLight(digit, 8, col);
  }
  if (val == 5) { // "5"
    //segments A,B,C,D,E,F
    segLight(digit, 1, col);
    segLight(digit, 2, col);
    segLight(digit, 3, 0);
    segLight(digit, 4, col);
    segLight(digit, 5, col);
    segLight(digit, 6, 0);
    segLight(digit, 7, col);
    //segLight(digit, 8, col);
  }
  if (val == 6) { // "6"
    //segments A,B,C,D,E,F
    segLight(digit, 1, col);
    segLight(digit, 2, col);
    segLight(digit, 3, 0);
    segLight(digit, 4, col);
    segLight(digit, 5, col);
    segLight(digit, 6, col);
    segLight(digit, 7, col);
    //segLight(digit, 8, col);
  }
  if (val == 7) { // "7"
    //segments A,B,C,D,E,F
    segLight(digit, 1, 0);
    segLight(digit, 2, col);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, 0);
    segLight(digit, 6, 0);
    segLight(digit, 7, 0);
    segLight(digit, 8, 0);
  }
  if (val == 8) { // "8"
    //segments A,B,C,D,E,F
    segLight(digit, 1, col);
    segLight(digit, 2, col);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, col);
    segLight(digit, 6, col);
    segLight(digit, 7, col);
    segLight(digit, 8, col);
  }
  if (val == 9) { // "9"
    //segments A,B,C,D,E,F
    segLight(digit, 1, col);
    segLight(digit, 2, col);
    segLight(digit, 3, col);
    segLight(digit, 4, col);
    segLight(digit, 5, col);
    segLight(digit, 6, 0);
    segLight(digit, 7, col);
    segLight(digit, 8, col);
  }
  if (val == 11) { // "2 center dots"
    //segments A,B,C,D,E,F
    segLight(digit, 1, 0);
    segLight(digit, 2, 0);
    segLight(digit, 3, 0);
    segLight(digit, 4, 0);
    segLight(digit, 5, 0);
    segLight(digit, 6, 0);
    segLight(digit, 7, 0);
    segLight(digit, 8, col);
  }
}
//END void digitWrite(int digit, int val, int col)
//////////////////////////////////////////////////


/////////////////////////////////////////////
void segLight(char digit, int seg, int col) {

  //'digit' picks which neopixel strip
  //'seg' calls a segment
  //'col' is color

  int color[3];

  //color sets
  if (col == 0) { //off
    color[0] = {0};
    color[1] = {0};
    color[2] = {0};
  }
  if (col == 1) { //red
    color[0] = {255};
    color[1] = {0};
    color[2] = {0};
  }
  if (col == 2) { //green
    color[0] = {0};
    color[1] = {255};
    color[2] = {0};
  }
  if (col == 3) { //blue
    color[0] = {0};
    color[1] = {0};
    color[2] = {255};
  }
  if (col == 4) { //white -- careful with this one, 3x power consumption
    color[0] = {255};
    color[1] = {255};
    color[2] = {255};
  }
  if (col == 5) { //yellow
    color[0] = {200};
    color[1] = {120};
    color[2] = {0};
  }
  if (col == 6) { //random  - gets lots of middle colurs nust be a better solution
    color[0] = {random(0, 255)};
    color[1] = {random(0, 255)};;
    color[2] = {random(0, 255)};
  }
  if (col == 7) { //purple
    color[0] = {255};
    color[1] = {255};;
    color[2] = {0};
  }


  //sets are 0-7,
  //8-15, 16-23, 24-31, 32-39, 40-47, 48-55 
  //sets are 0-19,
  //20-39, 40-59, 60-79, 80-99, 100-119, 120-139 //note number of leds willdepend on dencity of strip.

  //seg F
  //sets are f0-7, a8-15, b16-23, c24-31, d32-39, e40-47, g48-55, 56
  //new5     f
  //seg F
  if (seg == 1) {
    //light 15-19
    for (int i = 60; i <= 79; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg A
  if (seg == 2) {
    //light second 8
    for (int i = 40; i <= 59; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg B
  if (seg == 3) {
    for (int i = 20; i <= 39; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg C
  if (seg == 4) {
    for (int i = 120; i <= 139; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg D
  if (seg == 5) {
    for (int i = 100; i <= 119; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg E
  if (seg == 6) {
    for (int i = 80; i <= 99; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg G
  if (seg == 7) {
    for (int i = 0; i <= 19; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
  //seg dp // 2 center dots for clock
  if (seg == 8) {
    for (int i = 0; i <= 3; i++) {
      strip[digit].setPixelColor(i, color[0], color[1], color[2]);
    }
  }
}
//END void segLight(char digit, int seg, int col)
/////////////////////////////////////////////////

/////////////////////////////////////////////////
void adjustTime( NeoGPS::time_t & dt )
{
  NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds

  #ifdef CALCULATE_DST
    //  Calculate DST changeover times once per reset and year!
    static NeoGPS::time_t  changeover;
    static NeoGPS::clock_t springForward, fallBack;

    if ((springForward == 0) || (changeover.year != dt.year)) {

      //  Calculate the spring changeover time (seconds)
      changeover.year    = dt.year;
      changeover.month   = springMonth;
      changeover.date    = springDate;
      changeover.hours   = springHour;
      changeover.minutes = 0;
      changeover.seconds = 0;
      changeover.set_day();
      // Step back to a Sunday, if day != SUNDAY
      changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
      springForward = (NeoGPS::clock_t) changeover;

      //  Calculate the fall changeover time (seconds)
      changeover.month   = fallMonth;
      changeover.date    = fallDate;
      changeover.hours   = fallHour - 1; // to account for the "apparent" DST +1
      changeover.set_day();
      // Step back to a Sunday, if day != SUNDAY
      changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
      fallBack = (NeoGPS::clock_t) changeover;
    }
  #endif

  //  First, offset from UTC to the local timezone
  seconds += zone_offset;

  #ifdef CALCULATE_DST
    //  Then add an hour if DST is in effect
    if ((springForward <= seconds) && (seconds < fallBack))
      seconds += NeoGPS::SECONDS_PER_HOUR;
  #endif

  dt = seconds; // convert seconds back to a date/time structure

} // adjustTime
//////////////////////////////////////////////////////////////
