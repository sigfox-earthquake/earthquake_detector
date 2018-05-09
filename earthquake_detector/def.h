/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 *
 */

#define ACCEL_MODE 2        // 2G scale for the accelerometer
#define ACCEL_TRIG 40       // threshold for trigger

//--------------------------------------------------------------------------------------------//

#define BATT_MIN 3570
#define BATT_MAX 4200

// EEPROM map
#define ADDR_TODAY     0
#define ADDR_SENT      1 // byte01-31: number of messages sent
#define ADDR_CAL_LOW  32 // byte32-33: battery calibration
#define ADDR_CAL_HIGH 33

//GPS PMTK
//#define PMTK_SET_NMEA_OUTPUT_RMCGGA  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*" // 28"

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

boolean GPSactive = true;

uint8_t limitSport = 0;
uint8_t forceSport = 0;

unsigned long start = 0;
uint8_t  today = 0;
uint8_t  MsgCount = 0;

uint16_t batteryValue;
byte     batteryPercent = 0;

//accelerometer
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
  static void pulseHandler();         // handles pulse/tap event
};

