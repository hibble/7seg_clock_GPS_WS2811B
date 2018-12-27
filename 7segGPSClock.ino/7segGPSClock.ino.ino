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
int hours = 0;
int mins = 0;

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
//--------------------------------
//LED settings - UK colour used througout code not US spelling
//--------------------------------
const int ledbrightness = 40; //brightness for all pixels 0-255 range, 32 being dim
int dispColour = 7; // set RGB led colour not cons so can change e.g green if gps time red if no gps signal etc...


//--------------------------------
//Tasks scedualing
//--------------------------------
// Task timer settings
const int GPSMillis = 2000; //get GPS time every 2 sec (set to higher value once code is more meture e.g 1x evey 10min?)
const int DisplayMillis = 2000; // update display every 10th of a second
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
  delay(2000); //needed for some arduinos casiuses exrea 2 second startup time not a issue for a clock
  //while(!Serial); //for Leonardo & similar
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

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

  gpsPort.begin( 9600 );
  //initialize serial for debugging and monotoring
  //Serial.begin(9600); //neo-6m works on 9600. difrent gps may need other speeds
  //Serial.println("GPSClock TEST");

  //get tasks setup and enabled
  runner.init();
  runner.addTask(t1);
  runner.addTask(t2);
  runner.addTask(t3);
  t1.enable();
  t2.enable();
  t3.enable();
  
  //LEDStrip array setup
  for(int s=0;s<4;s++){ //s is number ofled strip digits in use. 
    strip[s].begin(); // Initialize pins for output
    strip[s].setBrightness(ledbrightness); //brightness 0-255
    strip[s].show();  // Turn all LEDs off 
    delay(200);
  }
  //flash dashes till we have gps time
    for(int t=0;t<4;t++){ // t is number ofled strip digits in use.
       digitWrite(t,8,0);//blank
       strip[t].show();
       segLight(t, 7, dispColour); //
       strip[t].show();
    }

  delay(1000);
}
//END void setup() stage
//////////////////

//--------------------------------
//Main Loop
//--------------------------------
void loop() {
  //main GPS code
   while (gps.available( gpsPort )) {
    fix = gps.read();
  }
  //run our update tasks
  runner.execute();
}
//END void loop()
/////////////////
void getGPSTime()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    //doSomeWork();
    gpsPort.println("loop");
  }
}
//END getGPSTime loop()
/////////////////

// Actually update the led strip 7 seg display.
// task called periodically by TaskScheduler
void update7segDisplay() {
     
    hours = fix.dateTime.hours; //probabkly better in gps loop 
    mins = fix.dateTime.minutes;
    int hoursTens = hours/10; //get the tens place of the hour
    int hoursOnes = hours%10; //get the ones place of the hour
    int minsTens = mins/10; //get the tens place of minutes
    int minsOnes = mins%10; //get the ones place of minutes

    //write to NeoPixel seven segments, colours:
    //0 = off
    //1 = red
    //2 = green
    //3 = blue
    //4 = white
    //5 =  yellow
    //
    digitWrite(0, minsOnes, dispColour); 
    strip[0].show();
    digitWrite(1, minsTens, dispColour);
    strip[1].show();
    digitWrite(2, hoursOnes, dispColour);
    strip[2].show();
    digitWrite(3, hoursTens, dispColour);
    strip[3].show();
    //debug 
    trace_all( DEBUG_PORT, gps, fix );
    Serial.print("Time:"); Serial.print(fix.dateTime.hours);
    Serial.print(":"); Serial.println(fix.dateTime.minutes);


}
//END void updateDisplay()
//////////////////////////

void updateSerialMonitor()
{
//some code here
}
//END void loop()
/////////////////

// shamless copy of other code see github for ninjaTimer
////////////////////////////////////////////////////////////////////////////////
void digitWrite(int digit, int val, int col){

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
  if (val==0){ // "0"
    //segments A,B,C,D,E,F
    segLight(digit,1,col);
    segLight(digit,2,col);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,col);
    segLight(digit,6,col);
    segLight(digit,7,0);
    segLight(digit,8,col);
  }
  if (val==1){ // "1"
    //segments A,B,C,D,E,F
    segLight(digit,1,0);
    segLight(digit,2,0);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,0);
    segLight(digit,6,0);
    segLight(digit,7,0);
    segLight(digit,8,col);
  }
  if (val==2){ // "2"
    //segments A,B,C,D,E,F
    segLight(digit,1,0);
    segLight(digit,2,col);
    segLight(digit,3,col);
    segLight(digit,4,0);
    segLight(digit,5,col);
    segLight(digit,6,col);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }
  if (val==3){ // "3"
    //segments A,B,C,D,E,F
    segLight(digit,1,0);
    segLight(digit,2,col);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,col);
    segLight(digit,6,0);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }
  if (val==4){ // "4"
    //segments A,B,C,D,E,F
    segLight(digit,1,col);
    segLight(digit,2,0);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,0);
    segLight(digit,6,0);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }
  if (val==5){ // "5"
    //segments A,B,C,D,E,F
    segLight(digit,1,col);
    segLight(digit,2,col);
    segLight(digit,3,0);
    segLight(digit,4,col);
    segLight(digit,5,col);
    segLight(digit,6,0);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }
  if (val==6){ // "6"
    //segments A,B,C,D,E,F
    segLight(digit,1,col);
    segLight(digit,2,col);
    segLight(digit,3,0);
    segLight(digit,4,col);
    segLight(digit,5,col);
    segLight(digit,6,col);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }          
  if (val==7){ // "7"
    //segments A,B,C,D,E,F
    segLight(digit,1,0);
    segLight(digit,2,col);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,0);
    segLight(digit,6,0);
    segLight(digit,7,0);
    segLight(digit,8,col);
  }
  if (val==8){ // "8"
    //segments A,B,C,D,E,F
    segLight(digit,1,col);
    segLight(digit,2,col);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,col);
    segLight(digit,6,col);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }
  if (val==9){ // "9"
    //segments A,B,C,D,E,F
    segLight(digit,1,col);
    segLight(digit,2,col);
    segLight(digit,3,col);
    segLight(digit,4,col);
    segLight(digit,5,col);
    segLight(digit,6,0);
    segLight(digit,7,col);
    segLight(digit,8,col);
  }    
}
//END void digitWrite(int digit, int val, int col)
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
void segLight(char digit, int seg, int col){ 

  //'digit' picks which neopixel strip
  //'seg' calls a segment
  //'col' is color

  int color[3];

  //color sets
    if (col==0){ //off
      color[0]={0};
      color[1]={0};
      color[2]={0};
    }
    if (col==1){ //red
      color[0]={255};
      color[1]={0};
      color[2]={0};
    }
    if (col==2){ //green
      color[0]={0};
      color[1]={255};
      color[2]={0};
    }
    if (col==3){ //blue
      color[0]={0};
      color[1]={0};
      color[2]={255};
    }
    if (col==4){ //white -- careful with this one, 3x power consumption
      color[0]={255};
      color[1]={255};
      color[2]={255};
    }
     if (col==5){ //yellow
      color[0]={200};
      color[1]={120};
      color[2]={0};
    }
    if (col==6){ //random
      color[0]={random(0,255)};
      color[1]={random(0,255)};;
      color[2]={random(0,255)};
    }
    if (col==7){ //purple
      color[0]={255};
      color[1]={255};;
      color[2]={0};
    }
  

  //sets are 0-7, 
  //8-15, 16-23, 24-31, 32-39, 40-47, 48-55 

  //seg F
//sets are f0-7, a8-15, b16-23, c24-31, d32-39, e40-47, g48-55, 56
  //new5     f
  //seg F
  if(seg==1){
    //light 15-19
    for(int i=15; i<=19; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
    }  
  }
  //seg A
  if(seg==2){
      //light second 8
      for(int i=10; i<=14; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      } 
  }
  //seg B
  if(seg==3){
      for(int i=5; i<=9; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      }   
  }
  //seg C
  if(seg==4){
      for(int i=30; i<=34; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      }   
  }
  //seg D
  if(seg==5){
      for(int i=25; i<=29; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      }   
  }
  //seg E
  if(seg==6){
      for(int i=20; i<=24; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      }   
  }
  //seg G
  if(seg==7){
      for(int i=0; i<=4; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      }   
  }
  //seg dp
  if(seg==8){
      for(int i=56; i<=57; i++){
      strip[digit].setPixelColor(i,color[0],color[1],color[2]);
      }   
  }
}
//END void segLight(char digit, int seg, int col)
////////////////////////////////////////////////////////////////////////////////
