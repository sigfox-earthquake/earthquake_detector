/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 *
 */

void gpsCmd (PGM_P s) {
  int XOR = 0;
  char c;
  while ((c = pgm_read_byte(s++)) != 0) {
    Serial.print(c);
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  if (DEBUG) {
    if (XOR < 0x10) Serial.print("0");
    Serial.println(XOR, HEX);
  }
}

bool gpsInit()
{
  boolean GPSready = false;
  digitalWrite(rstPin, HIGH);
  unsigned long startloop = millis();
  while ((uint16_t) (millis() - startloop) < 6000 ) {
    if (Serial.available() > 0 && Serial.read() == '*') {
      GPSready = true;
      break;
    }
    delay(100);
  }
  if (GPSready) {
    if (DEBUG) {Serial.println("GPS ready");}
    gpsCmd(PSTR(PMTK_SET_NMEA_OUTPUT_RMCGGA));
    gpsCmd(PSTR(PMTK_SET_NMEA_UPDATE_1HZ));   // 1 Hz update rate
  } else digitalWrite(rstPin, LOW);
  return GPSready;
}

void gpsStandby() {
  GPSactive = false;
  digitalWrite(rstPin, LOW);
}

int gpsProcess()
{
  boolean newGpsData = false;
  boolean newSerialData = false;
  float distance;
  unsigned long start = millis();
  unsigned int waitime = 2000;
  // Parse GPS data for 2 second
  while ((uint16_t) (millis() - start) < waitime)
  {
    if (Serial.available() > 0) {
      newSerialData = true;
      waitime = 100;
      start = millis();
//      redLEDon;
    }
    while (Serial.available())
    {
      char c = Serial.read();
      // New valid NMEA data available
      if (gps.encode(c)) newGpsData = true;
    }
  }
  // Check if NMEA packet received, wake up GPS otherwise
  if (!newSerialData) gpsInit();

  // Get date/time info and store in datetime struct
  gps.crack_datetime( &gps_fix.datetime.year, &gps_fix.datetime.month,
                      &gps_fix.datetime.day, &gps_fix.datetime.hour,
                      &gps_fix.datetime.minute, &gps_fix.datetime.second,
                      &gps_fix.datetime.hundredths, &fix_age);
  timeLastGPS = millis();

  // Get GPS info and store in p struct
  if (newGpsData) { // computeData
    gps.f_get_position(&p.lat, &p.lon, &fix_age);
    sat = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    alt = abs(round(gps.f_altitude()));
    spd = round(gps.f_speed_kmph());
    syncSat += sat;
    noSat = 0;
  }
  else noSat++;

  printData(newGpsData); // For debug purpose

//printData(true);

///if no data return 0 else if newSerialData && newGpsData return 1 else if newSerialData return 2 else if newGpsData return 3
  //return newSerialData;
  if (!newSerialData && !newGpsData) return 0;
  else if (newSerialData && newGpsData) return 1;
  else if (newSerialData) return 2;
  else if (newGpsData) return 3;
}

void print_date()
{
  char sz[24];
  sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
          gps_fix.datetime.month, gps_fix.datetime.day, gps_fix.datetime.year, gps_fix.datetime.hour, gps_fix.datetime.minute, gps_fix.datetime.second);
  Serial.print(sz);
}

void printData(bool complete) {
  print_date();
  if (complete) {
    serialString(PSTR("syncSat="));
    Serial.print(syncSat);
    serialString(PSTR(", lat="));
    Serial.print(p.lat, 7);
    serialString(PSTR(", lon="));
    Serial.print(p.lon, 7);
    serialString(PSTR(", alt="));
    Serial.print(alt);
    serialString(PSTR(", cap="));
    Serial.print((gps.course() / 90) % 4);
    serialString(PSTR(", spd="));
    Serial.print(spd);
    serialString(PSTR(", sat="));
    Serial.print(sat);
  } else {
    serialString(PSTR(" noSat="));
    Serial.print(noSat);
  }
  if (forceSport) {
    serialString(PSTR(", sport"));
  }
  serialString(PSTR(", bat="));
  Serial.println(batteryPercent);
  Serial.flush();
}

