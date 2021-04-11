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
#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
// Uncomment to run example with custom SPI and SS e.g with FRAM breakout
// #define CUSTOM_CS   A5
// #define CUSTOM_SPI  SPI

#if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
  Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);

#elif CONFIG_IDF_TARGET_ESP32S2
  // ESP32-S2 use same flash device that store code.
  // Therefore there is no need to specify the SPI and SS
  Adafruit_FlashTransport_ESP32 flashTransport;

#else
  // On-board external flash (QSPI or SPI) macros should already
  // defined in your board variant if supported
  // - EXTERNAL_FLASH_USE_QSPI
  // - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;

  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

  #else
    #error No QSPI/SPI flash are defined on your board variant.h !
  #endif
#endif

#define HT_CHECK 18288; //height in meters that the 

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;

File myFile;

SFE_UBLOX_GNSS myGNSS;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

bool enableHeater = false;
uint16_t loopCnt = 0;
//Setting this to something high so that it will run for enough times
long altitudeMSL = 100000;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

Adafruit_BMP280 bmp; // use I2C interface
//Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor(); don't need this because we are getting the temp from the other sensor...
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

bool debug = true;

bool isFileOpen = true;

void setup()
{
  if(debug){
    Serial.begin(115200);
    while (!Serial) {
      delay(1); // wait for serial port to connect. Needed for native USB port only
    }
  }
  

  Wire.begin();
  //GPS setup
  if (myGNSS.begin() == false)
  {
    if(debug) Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //Environmental Sensors setup
  if(debug) Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    if(debug) Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  if(debug) Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled()){
    if(debug) Serial.println("ENABLED");
  }
  else{
    if(debug) Serial.println("DISABLED");
  }
    

  if (!bmp.begin()) {
    if(debug) Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //Starting file to write
  if(debug) Serial.print("Initializing Filesystem on external flash...");
  
  // Init external flash
  flash.begin();

  // Open file system on the flash
  if ( !fatfs.begin(&flash) ) {
    if(debug) Serial.println("Error: filesystem is not existed. Please try SdFat_format example to make one.");
    while(1) yield();
  }

  if(debug) Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = fatfs.open("GPS_ENV.txt", FILE_WRITE);

  // if the file opened okay, write to it:
//  if (myFile) {
//    Serial.print("Writing to test.txt...");
//    myFile.println("testing 1, 2, 3.");
//    // close the file:
//    myFile.close();
//    Serial.println("done.");
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//  }
//
//  // re-open the file for reading:
//  myFile = fatfs.open("test.txt");
//  if (myFile) {
//    Serial.println("test.txt:");
//
//    // read from the file until there's nothing else in it:
//    while (myFile.available()) {
//      Serial.write(myFile.read());
//    }
//    // close the file:
//    myFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//  }
  
}
  


void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
  //check to see how long this has been running, we don't want to run and save this super often 
  //if it has been running for at least 2 hours AND the altitude is less than 500 meters (average Iowa height is 335 meters)
  // AND if the file has not been closed yet, run the file.
//  if(loopCnt < 720 && altitudeMSL < 500 && isFileOpen){
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
      altitudeMSL = myGNSS.getAltitudeMSL()/1000;
      byte fixType = myGNSS.getFixType();
      byte RTK = myGNSS.getCarrierSolutionType();
      int Sats = nmea.getNumSatellites();
      if(debug){
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

      //I'm assuming that the File object's "print()" and "println" work the same way as the Serial port
      myFile.print("GPS:");
      myFile.print(latitude_mdeg / 1000000., 6);
      myFile.print(":");
      myFile.print(longitude_mdeg / 1000000., 6);
      myFile.print(":");
      myFile.print(Sats);
      myFile.print(":");
      myFile.print(altitudeMSL);
      myFile.print(":");
      if(fixType == 0) myFile.print(F("NF:"));
      else if(fixType == 1) myFile.print(F("DR:"));
      else if(fixType == 2) myFile.print(F("2D:"));
      else if(fixType == 3) myFile.print(F("3D:"));
      else if(fixType == 4) myFile.print(F("GNSS:"));
      else if(fixType == 5) myFile.print(F("Time:"));
      if (RTK == 0) myFile.print(F("0:"));
      else if (RTK == 1) myFile.print(F("1:"));
      else if (RTK == 2) myFile.print(F("2:"));
      myFile.println("END");
    }
  
    
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
  
    if (! isnan(t)) {  // check if 'is not a number'
      if(debug) {
        Serial.print("Temp *C = "); 
        Serial.print(t); 
        Serial.print("\t\t");
      }
      myFile.print("Temp *C = "); 
      myFile.print(t); 
      myFile.print("\t\t");
    } 
    else { 
      if(debug) Serial.println("Failed to read temperature");
    }
    
    if (! isnan(h)) {  // check if 'is not a number'
      if(debug) {
        Serial.print("Hum. % = "); 
        Serial.println(h);
      }
      myFile.print("Hum. % = "); 
      myFile.println(h);
    } 
    else { 
      if(debug) Serial.println("Failed to read humidity");
    }
    
    sensors_event_t pressure_event;
    bmp_pressure->getEvent(&pressure_event);
    if(debug){
      Serial.print(F("Pressure = "));
      Serial.print(pressure_event.pressure);
      Serial.println(" hPa");
      Serial.println();
    }
    myFile.print(F("Pressure = "));
    myFile.print(pressure_event.pressure);
    myFile.println(" hPa");
    myFile.println();

    //if we are still less than 60000 ft (~18250 m) then only take measurements every 10 seconds
//    if(altitudeMSL < HT_CHECK){
      delay(10000);
//    }
    //otherwise, take measurements every 1 second
//    else{
//      delay(1000);
//    }
    
//  }
//  else{
//    myFile.close();
//    isFileOpen = false;
//  }
  loopCnt++; 
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
