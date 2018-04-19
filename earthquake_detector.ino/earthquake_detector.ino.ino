/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 * 
 */

#include "LowPower.h"
#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include "Wire.h"
#include "mma8652.h"

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

// Info
 


#define DEBUG 1
// Variables


// Sigfox
SoftwareSerial Sigfox =  SoftwareSerial(txSigfox, rxSigfox);
uint8_t msg[12];

// GPS
//SoftwareSerial GPS =  SoftwareSerial(txGPS, rxGPS);

// Accelerometer

void setup() {
  if(DEBUG){
    Serial.begin(9600);
    delay(100);
  } 
  
  Sigfox.begin(9600);
  delay(100);
  getID();
  delay(100);
  
  GPS.begin(4800);
  delay(100);

  //Declare other variables

}

void loop() {
  
}


//Send Sigfox Message
void sendMessage(uint8_t msg[], int size){

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
     blink();
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
     blink();
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

// Blink can be used for debug
void blink(){
  digitalWrite(bluLEDpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(bluLEDpin, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}
