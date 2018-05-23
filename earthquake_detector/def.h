/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 *
 */

 // Offset hours from gps time (UTC)?
//const int offset = 1;   // Central European Time
//const int offset = -5;  // Eastern Standard Time (USA)
//const int offset = -4;  // Eastern Daylight Time (USA)
//const int offset = -8;  // Pacific Standard Time (USA)
//const int offset = -7;  // Pacific Daylight Time (USA)

//modifiable variables

#define DEBUG 1                  // 1 = DEBUG mode, 0 = regulare

#define CLOCK_FIX_LOOP 60        //Time in minutes to recalibrate clock based on GPS
#define DEVICE_SLEEP_LOOP 60     //Time in seconds to recalibrate clock based on GPS

#define MASS 45             //mass of device

#define ACCEL_MODE AFS_2g   // AFS_(2,4,8)g scale for the accelerometer
#define ACCEL_TRIG 3        // threshold (0-127) for trigger (TRANSIENT interrupt)

#define SLP_CNT 5          //sleep count in approximate seconds after interrupt max 85
#define DATA_RATE AODR_50HZ //Accel output data rate (Highest-lowest)
                            //(AODR_800HZ, AODR_400HZ, AODR_200HZ, AODR_100HZ, AODR_50HZ, AODR_12_5HZ, AODR_6_25HZ, AODR_1_56HZ)
#define SLEEP_DATA_RATE B11 ////Asleep data rate for low power 11 = 1.56Hz see Datasheet Table 95, p.54

//--------------------------------------------------------------------------------------------//

#define BATT_MIN 3570
#define BATT_MAX 4200

// EEPROM map
#define ADDR_TODAY     0
#define ADDR_SENT      1 // byte01-31: number of messages sent
#define ADDR_CAL_LOW  32 // byte32-33: battery calibration
#define ADDR_CAL_HIGH 33

//////////////added for GPS because of wierd errors with battery////////////////
#define PMTK_SET_NMEA_OUTPUT_RMCGGA  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*" // 28"

#define PMTK_AWAKE   "$PMTK010,001*" // 2E"
#define PMTK_STANDBY "$PMTK161,0*"   // 28"
#define PMTK_VERSION "$PMTK605*"     // 31"
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*" // 1F"
#define PMTK_ENABLE_SBAS "$PMTK313,1*" // 2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*" // 2E"
////////////////////////////////////////////////////////

/****************** Pins usage ***************************************/
#define rxGPS            0     // PD0 RX Serial from GPS
#define txGPS            1     // PD1 TX Serial to GPS
#define usbDP            2     // PD2 Shutdown supply after power off
#define accINT           3     // PD3 Accelerometer Interruption
#define usbDM            4     // PD4
#define txSigfox         5     // PD5 TX Serial to Sigfox modem
#define bluLEDpin        6     // PD6 Piezzo Output
#define redLEDpin        7     // PD7 Red LED Status
#define rxSigfox         8     // PB0 RX Serial from Sigfox modem
#define shdPin           9     // PB1 Shutdown pin
#define rstPin          10     // PB2 SS   SDCARD
#define msiPin          11     // PB3 MOSI SDCARD
#define msoPin          12     // PB4 MISO SDCARD
#define sckPin          13     // PB5 SCK  SDCARD
#define sensorA0        A0     // PC0 VUSB present
#define sensorBatt      A1     // PC1 battery voltage
#define chg100mA        A2
#define satLEDpin       A3
#define sensorA4        A4     // PC4 A4 SDA
//                      A5     // PC5 A5 SCL
#define sensorA6        A6     // PC6
#define chgFLAG         A7
/*********************************************************************/

/****************** Pins output values *******************************/
#define DIGITAL_PULLUP ((1 << shdPin) | (1 << accINT) | (1 << usbDP) | (1 << usbDM))
/*********************************************************************/

/****************** Pins direction ***********************************/
#define DDRC_MASK (1 << 2)
#define DIGITAL_OUTPUT ((1 << shdPin) | (1 << redLEDpin) | (1 << bluLEDpin) | (1 << rxSigfox) | (1 << rstPin))
/*********************************************************************/

///////time
// macros from DateTime.h 
/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  
/////////

int largest_x = 0;
int largest_y = 0;
int largest_z = 0;

boolean GPSactive = true;
unsigned long fix_age = 0;

uint8_t limitSport = 0;
uint8_t forceSport = 0;

unsigned long start = 0;
unsigned long timeLastGPS = 0;
unsigned long int initial_time = 0;
uint8_t  today = 0;
uint8_t  MsgCount = 0;

uint16_t batteryValue;
byte     batteryPercent = 0;
unsigned int sensorMax;


volatile uint8_t button_flag = 0;
volatile boolean button_pressed = false;

//accelerometer
static uint32_t timeLastTransient = 0;
class InitializeFluMMA865x
{
public:
  static void begin();  // Prepare, Calibrate, Initialize
  static void init();  //  Initialize MMA865x accelerometer config registers
};

class Accel
{
public:
  static void work();               // called by main loop(), prints any occured interrupt, checks if accel data is ready
  static void accelDataHandler();   // gets and prints accel data
  static void transientHandler();   // handles transient event = transient IRQ
};

