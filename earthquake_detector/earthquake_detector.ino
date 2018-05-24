/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 * 
 */

#include "EEPROM.h"
#include "def.h"
#include "HidnSeek.h"
#include "LowPower.h"
#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include "Wire.h"

// Accelerometer
#include <Streaming.h>
#include <FluMMA865xI2C.h>        // Accelerometer I2C bus communication
#include <FluMMA865xR.h>          // Accelerometer configuration and data register layout
#include <FluMMA865x.h>           // Accelerometer basic utility functions
#include <AccelDataT.h>           // A 3D Vector data type for x/y/z data

uint16_t                   convFactMicrograv; // factor for conversion from lsb to micrograv
AccelDataT<int16_t>        accelLsb;
FluMMA865xR::IntSourceRegT lastIntSourceR;

FluMMA865x           accel;
FluMMA865xI2C        comms;
InitializeFluMMA865x init_accel;
Accel                accelero;

HidnSeek HidnSeek(txSigfox, rxSigfox);/////not sure if we need this (could just add code for GPIOinit())

// Info

// Variables in def.h

   //Sigfox
uint8_t msg[12];

    //GPS
    //Position structure, data stored as degrees, decimals minutes
  typedef struct {
    int32_t latitude;       ///< Latitude in 1/100000 degrees, South if < 0, North otherwise
    int32_t longitude;      ///< Longitude in 1/100000 degrees, West if < 0, East otherwise
    int16_t altitude;       ///< Altitude in meters above sea level
  } GPS_Position;

  //Date & Time structure
  typedef struct {
    uint16_t year = 0;          ///< Year UTC
    uint8_t hundredths = 0;     //
    uint8_t second = 0;         ///< Seconds 0..59
    uint8_t minute = 0;         ///< Minutes 0..59
    uint8_t hour = 0;           ///< Hours 0..23
    uint8_t day = 0;            ///< Days 1.31
    uint8_t month = 0;          ///< Month 1..12
  } GPS_DateTime;

  // Quality structure
  typedef struct {
    uint16_t hdop;          ///< Horizontal Degree Of Precision, 0 .. 20 * 100
    uint16_t hAcc;          ///< Horizontal estimated Accuracy in meters
    uint8_t sat;            ///< Number of satellites used for fix, maximum 20
  } GPS_Quality;

  // GPS fix structure types
  typedef enum {
    TD_GEOLOC_NO_FIX = 0,
    TD_GEOLOC_TIME_FIX = 1,
    TD_GEOLOC_DATE_FIX = 2,
    TD_GEOLOC_2D_FIX = 3,
    TD_GEOLOC_3D_FIX = 4
  } GPS_FixType;

  //Geoloc information saved in logger */
  typedef struct {
    GPS_Position position;  ///< Position information
    GPS_DateTime datetime;     ///< Date & time information
  } GPS_Log;

  // Full structure containing all updated informations
  typedef struct {
    GPS_DateTime datetime;    ///< Date & time information
    GPS_Position position;    ///< Position information
    GPS_Quality quality;      ///< Quality information
    uint32_t speed;           ///< Speed information (in km/h)
    GPS_FixType type;         ///< Type of fix information available
    uint32_t duration;        ///< Fix duration in seconds
  } GPS_Fix;

  //Accelero

  //Battery

// Sigfox
SoftwareSerial Sigfox =  SoftwareSerial(txSigfox, rxSigfox);

// GPS
TinyGPS gps;///////added
SoftwareSerial GPS =  SoftwareSerial(txGPS, rxGPS);
GPS_Fix gps_fix;

// Accelerometer


struct Payload {
  float lat;
  float lon;
  uint32_t cpx;
};

Payload p;
uint16_t alt = 0;
uint16_t spd = 0;
uint8_t  sat = 0;
uint8_t  syncSat = 0;
uint8_t  noSat = 0;

// RAM check
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Everytime interrupt, this function can be called
void interruptFuction()
{
  // Set previous largest values back to 0
  largest_x = largest_y = largest_z = 0;
}

// Everytime interrupt, this function can be called
void sleepFuction()
{
  if (DEBUG){
    Serial.println((String)"LARGEST x: " + largest_x + " y: " + largest_y + " z: " + largest_z);
    Serial.println((String)"Start: " + start);
    Serial.println(millis()/1000);
  }
  calibrateClock();
  if (DEBUG)
  {
    print_date();
    Serial.println();
  }
}

void wakeUp()
{
  if (DEBUG) Serial.println("Wake");
//  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void setup() {
  if(DEBUG){
    Serial.begin(9600);
    Serial.println("Debug mode");
    delay(100);
  }
  
  Sigfox.begin(9600);
  delay(100);
  getID();
  delay(100);
  
  GPS.begin(4800);
  delay(100);

  //Declare other variables
  pinMode(DIGITAL_OUTPUT, OUTPUT);
  pinMode(DIGITAL_PULLUP, INPUT_PULLUP);

  //init GPIO
  HidnSeek.initGPIO(false);

  //init battery
  init_battery();

  //init accelerometer
  init_accel.begin();

  //GPS, RAM
  setupDevice();

  //Set start millis() and initial time offset for clock calibration
  start = millis();
  initial_time =  (gps_fix.datetime.day * SECS_PER_DAY) +
                  (gps_fix.datetime.hour * SECS_PER_HOUR) +
                  (gps_fix.datetime.minute * SECS_PER_MIN) +
                  gps_fix.datetime.second - (start/1000);
  if (DEBUG){ 
    Serial.println(start);
    calibrateClock();
    print_date();
    Serial.println();
  }
}

void loop() {
  // Get accelerometer info
  accelero.work();
  
  // Hour loop to calibrate clock based on GPS
  if(millis() - timeLastGPS > (CLOCK_FIX_LOOP * SECS_PER_MIN * 1000)){
    if (DEBUG) Serial.println("Recalibrate clock/gps");
    // Get GPS info
    startGPSFix();
    delay(10);
    start = millis();
    initial_time =  (gps_fix.datetime.day * SECS_PER_DAY) +
                    (gps_fix.datetime.hour * SECS_PER_HOUR) +
                    (gps_fix.datetime.minute * SECS_PER_MIN) +
                    gps_fix.datetime.second - (start/1000);
  }

  // Put device to sleep after x amount of time
//  sleep_device();
}

/***************************************************************************//**
 * @brief
   This function is used to sleep the hidnseek device
   Sleeps after DEVICE_SLEEP_LOOP amount of seconds
   (Need to attach an interrupt to be able to wake device from accINT)
 *
 * @param[in] none at the moment

 ******************************************************************************/
static void sleep_device()
{
  // Put device to sleep after x amount of time after the last interrupt
  if(millis() - timeLastTransient > DEVICE_SLEEP_LOOP * 1000){
    if (DEBUG) Serial.println("Hidenseek sleeping");
    delay(100);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

/***************************************************************************//**
 * @brief
   This function is used to setup the battery
   It prints the value and percentage if in DEGUG mode
 *
 * @param[in] none at the moment

 ******************************************************************************/
static void init_battery()
{
  initSense();
  batterySense();
  if (DEBUG) 
  {
    serialString(PSTR(" Battery: "));
    Serial.print(batteryPercent);
    serialString(PSTR("% "));
    Serial.println(batteryValue);
  }
  delay(100);
}

/***************************************************************************//**
 * @brief
   This function is used to setup the device in the field.
   It must get the GPS information: Lat, Long, Alt and store them in memory
   It must sync the internal clock
   If the GPS acquisition went well, it must send a Sigfox message with:
   - The message type indicator (setup)
   - The battery level
   - The lat, long, alt
   If it went wrong (no GPS information for instance):
   - The message type indicator
   - The battery level
 *
 * @param[in] none at the moment

 ******************************************************************************/
static void setupDevice(){
  // Print availible RAM
  if (DEBUG)
  {
    serialString(PSTR("free Ram: "));
    Serial.println(freeRam());
  }

  // Get GPS info
  startGPSFix();

  // Send message if succesfull
// sendMessage(0x4, 8);
//  if (GPSactive = gpsInit()) {
//    gpsCmd(PSTR(PMTK_VERSION));
//    blink(2);
//  }
}

/***************************************************************************//**
 * @brief
 *   Start the GPS and try to get a fix. If the GPS is already running then no
 *   effect.
 *
 * @param[in] timeout
 *  timeout in seconds. On timeout expiration the timeout parameter of the
 *  callback will be set to true. No further action is being performed and
 *  shutting down the GPS or not is left to the user application and should be
 *  done using GPS_StopFix.
 *
 * @param[in] callback
 *   Pointer to a function that will be called each time a parameter
 *   is being updated (time, position, speed, etc..).
 *   Fix is a pointer to a structure containing fix information.
 *   Timeout will be set to true when the specified timeout expired
 *
 *
 ******************************************************************************/
void startGPSFix(){//uint16_t timeout, void (*callback)(GPS_Fix *fix, bool timeout)){
  while (!gpsProcess())
    if (DEBUG) Serial.println("Not calibrated");
  gpsProcess();
}


/***************************************************************************//**
 * @brief
 *  Power down the GPS
 *
 * @param[in] none at the moment
 ******************************************************************************/
static void StopGPSFix(){
  
}


/***************************************************************************//**
 * @brief
 *  GPS fix callback
 *
 * @param[in] fix
 *   The GPS fix data structure.
 *
 * @param[in] timeout
 *   Flag that indicates whether a timeout occurred if set to true.
 ******************************************************************************/
static void GPSFix(GPS_Fix * fix, bool timeout){
  //
  //
  
  //At the end stop GPS
  StopGPSFix();
}

/***************************************************************************//**
 * @brief
 *  Calibrate internal clock
 *
 * @param[in] fix
 *   The GPS fix data structure.
 *
 * @param[in] timeout
 *   Flag that indicates whether a timeout occurred if set to true.
 ******************************************************************************/

void calibrateClock(){
  // Resets initial_time when you check the gps again
  long val = (millis()/1000) + initial_time + (offset * SECS_PER_HOUR);
  //Serial.println((String)"val: " + val + " initial_time: " + initial_time + " millis: " + millis());
  //Serial.println();
  gps_fix.datetime.day = elapsedDays(val);
  gps_fix.datetime.hour = numberOfHours(val);
  gps_fix.datetime.minute = numberOfMinutes(val);
  gps_fix.datetime.second = numberOfSeconds(val);
}

// Send Sigfox Message
void sendMessage(uint8_t msg[], int size){
  // This function is used to send the Sigfox messages
  
  String status = "";
  char output;

  Sigfox.print("AT$SS=");
  for(int i= 0;i<size;i++){
    Sigfox.print(String(msg[i], HEX));
    if(DEBUG){
      Serial.print("Byte:");
      Serial.println(msg[i], HEX);
    }
  }

  Sigfox.print("\r");

  while (!Sigfox.available()){
     blink(1);
  }
  while(Sigfox.available()){
    output = (char)Sigfox.read();
    status += output;
    delay(10);
  }
  if(DEBUG){
    Serial.println();
    Serial.print("Status \t");
    Serial.println(status);
  }
}

//Get Sigfox ID
String getID(){
  String id = "";
  char output;

  Sigfox.print("ATI7\r");
  while (!Sigfox.available()){
     blink(1);
  }
  while(Sigfox.available()){
    output = Sigfox.read();
    id += output;
    delay(10);
  }
  if(DEBUG){
    Serial.println("Sigfox Device ID: ");
    Serial.println(id);
  }
  return id;
}

// Helper functions
// Blink can be used for debug
void blink(bool c){
  
  if (c == 1) {
    digitalWrite(bluLEDpin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(bluLEDpin, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);
  }
  else {
    digitalWrite(redLEDpin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(redLEDpin, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);
  }
}

void serialString (PGM_P s) {

  char c;
  while ((c = pgm_read_byte(s++)) != 0)
    Serial.print(c);
}

void saveEEprom() {
  if (today > 0 && today < 32) {
    EEPROM.write(today, MsgCount);
  }
}

