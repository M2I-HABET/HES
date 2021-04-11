/*
 * Backup Emergency Recovery Transmitter
 * HW Version - 3.0
 * FW Version - 1.0
 * Matthew E. Nelson
 */

/*
 * Some code based on the following Libraries
 * - Sparkfun GNSS Library
 * - BME680 Library
 * - MicroNMEA
 */

/*
  Read NMEA sentences over I2C using u-blox module SAM-M8Q, NEO-M8P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
  This example reads the NMEA characters over I2C and pipes them to MicroNMEA
  This example will output your current long/lat and satellites in view
 
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106
  For more MicroNMEA info see https://github.com/stevemarple/MicroNMEA
  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Go outside! Wait ~25 seconds and you should see your lat/long
*/

#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
#include <Arduino.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_BMP280.h>

SFE_UBLOX_GNSS myGNSS;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

bool enableHeater = false;
uint8_t loopCnt = 0;

Adafruit_SHT31 sht31 = Adafruit_SHT31();



Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  
}
  


void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
  /*
   * GPS Processing
   * String has the following format
   * GPS:LAT:LON:ALTMSL:FIXTYPE:#SATS:RTK:END
   * LAT/LON in Deg
   * Altitude MSL in meters
   * FIXTYPE
   * ============
   * 0 - No Fix
   * 1 - Dead Reckoning
   * 2 - 2D
   * 3 - 3D
   * 4 - GNSS + Dead Reckoning
   * 5 - Time info only
   * 
   * RTK
   * ==========
   * 1 - No Solution
   * 2 - High precision floating fix
   * 3 - High precision fix
   */

  if(nmea.isValid() == true)
  {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    //long altitudeGPS = myGNSS.getAltitude()/1000;
    long altitudeMSL = myGNSS.getAltitudeMSL()/1000;
    byte fixType = myGNSS.getFixType();
    byte RTK = myGNSS.getCarrierSolutionType();
    int Sats = nmea.getNumSatellites();
    Serial.print("GPS:");
    Serial.print(latitude_mdeg / 1000000., 6);
    Serial.print(":");
    Serial.print(longitude_mdeg / 1000000., 6);
    Serial.print(":");
    Serial.print(Sats);
    Serial.print(":");
    Serial.print(altitudeMSL);
    Serial.print(":");
    if(fixType == 0) Serial.print(F("NF:"));
    else if(fixType == 1) Serial.print(F("DR:"));
    else if(fixType == 2) Serial.print(F("2D:"));
    else if(fixType == 3) Serial.print(F("3D:"));
    else if(fixType == 4) Serial.print(F("GNSS:"));
    else if(fixType == 5) Serial.print(F("Time:"));
    if (RTK == 0) Serial.print(F("0:"));
    else if (RTK == 1) Serial.print(F("1:"));
    else if (RTK == 2) Serial.print(F("2:"));
    Serial.println("END");
  }

  
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
  
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.println();
    
  delay(10000);
 
}

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}
