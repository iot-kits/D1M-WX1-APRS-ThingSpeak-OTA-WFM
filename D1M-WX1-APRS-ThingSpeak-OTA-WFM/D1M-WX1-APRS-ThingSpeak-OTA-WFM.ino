// D1M-WX1-APRS-ThingSpeak-OTA-WFM.ino
const int    FW_VERSION = 1017;
const String FW_FILENAME = "D1M-WX1-APRS-ThingSpeak-OTA-WFM";

// TODO: fix adc autocalibrate, prevent out of range

// 2022-06-16 v1017 fixed sensor failure detect, removed sensorData fails, added APRS message
// 2022-06-15 v1016 bypass adcCalibrate, change to LittleFS, add DEBUG print, fix BITS: & PARAM:
//                  changed to Adafruit_BME280
// 2020-07-07 v1015 fix adcCalibrate - maybe
// 2020-07-07 v1014 combine initSensors & checkAlarms into readSensors
//                  revert to claws BH1750 library
//                  obfuscate OTA
// 2020-05-14 v1013 changed Tier 2 server to noam, improved adcCalibrrate
// 2020-05-14 v1012 sleep 10 minute OTA 60 minutes
// 2020-05-11 add autocalibrate ADC - test reset detector
// 2020-05-10 fix all APRSpad functions. Increment sequence logically. Limit timeAwake.
// 2020-05-09 Offset RTC to avoid conflict with OTA
// 2020-05-08 WiFiManager working, ThingSpeak working, OTA working

/*_____________________________________________________________________________
   Copyright(c) 2018-2022 Karl W. Berger dba IoT Kits https://w4krl.com/iot-kits

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   _____________________________________________________________________________
*/

// *******************************************************
// ************ COMPILER INCLUDES & MACROS ***************
// *******************************************************

// For WiFiManager
#include <LittleFS.h>                 // [builtin] File system for custom parameters
#include <ESP8266WebServer.h>         // [builtin] For captive portal
#include <WiFiManager.h>              // [manager] v0.16.0 by tzapu https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>              // [builtin] Wi-Fi
#include <ArduinoJson.h>              // [manager] v6.19.4 by Benoît Blanchon https://github.com/bblanchon/ArduinoJson

// For general sketch
#include <Wire.h>                     // [builtin] I2C bus
#include <BH1750.h>                   // [manager] v1.3.0 by Christopher Laws https://github.com/claws/BH1750
#include <Adafruit_BME280.h>          // [manager] v2.2.2 by Adafruit https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_Sensor.h>          // [manager] installed with Adafruit_BME280

// For HTTP OTA
#include <ESP8266HTTPClient.h>        // [builtin] http
#include <WiFiClientSecureBearSSL.h>  // [builtin] https
#include <ESP8266httpUpdate.h>        // [builtin] OTA

// For Double Reset Detector
#include <DoubleResetDetector.h>      // [manager] v1.0.3 by Stephen Denne https://github.com/datacute/DoubleResetDetector

//! uncomment this line for serial output
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// *******************************************************
// ******* Set true to erase config data *****************
// *******************************************************
// USE ONLY IF ALL ELSE FAILS
// If your device is confused or constantly rebooting:
// 1. Set RESET_WIFI = true;
// 2. Upload the firmware:
//    - DO NOT DO ANYTHING ELSE
//    - DO NOT CONFIGURE YOUR DEVICE
//    - Let it run ONCE
// 3. Set RESET_WIFI = false; and reupload.
// 4. Now you can do the normal configuration process.
const boolean RESET_WIFI = false;     // erases WiFI & LittleFS settings

// *******************************************************
// ******************* DEFAULTS **************************
// *******************************************************
//           CHANGEABLE DEFAULTS - CHANGE AT YOUR RISK
const long   SLEEP_INTERVAL = 600;    // must be 15 seconds or more
const int    OTA_SPAN = 6 * 60 * 60;  // seconds between OTA checks
const int    APRS_DEFINITION_SPAN = 4 * 60 * 60; // interval between APRS deifitions
const long   MIN_RSSI = -85;          // warning level for low WiFi signal strength
const float  MIN_VCELL = 3.2;         // warning level for low cell voltage

// !!!!!!    DO NOT CHANGE THESE DEFAULTS       !!!!!!
const String THINGSPEAK_SERVER = "api.thingspeak.com";    // ThingSpeak Server
const char   AP_PORTAL_NAME[] = "D1M-WX1-portal";         // Captive Portal name
const String CONFIG_FILENAME = "/config.json";            // LittleFS filename
const int    OTA_OFFSET = 32;                             // lower RTC memory used by OTA
// for list of tier 2 servers: http://www.aprs2.net/
const String APRS_SERVER = "noam.aprs2.net";              // recommended for North America
const String APRS_DEVICE_NAME = "https://w4krl.com/iot-kits/";
const String APRS_SOFTWARE_NAME = "D1M-WX1";              // unit ID
const String APRS_SOFTWARE_VERS = String(FW_VERSION);     // FW version
const int    APRS_PORT = 14580;                           // do not change port
const long   APRS_TIMEOUT = 2000;                         // milliseconds
const String APRS_PROJECT = "Solar Power WX Station";     // telemetry ID

// *******************************************************
// ******************* GLOBALS ***************************
// *******************************************************
bool saveConfigFlag = false;
String unitStatus = "";               // holds device error messages
long startTime = millis();            // record time at start of sketch
const int ADC_PIN = A0;               // voltage sense

// user parameter values entered through WiFiManager
const char* wm_callsign;              // callsign with ssid
const char* wm_passcode;              // APRS passcode
const char* wm_latitude;              // decimal latitude
const char* wm_longitude;             // decimal longitude
const char* wm_elevation;             // elevation in meters
const char* wm_channel;               // ThingSpeak channel ID
const char* wm_write_key;             // ThingSpeak API Write Key
const int USER_PARAMS = 7;            // set to number of user parameters for json memory

// String versions
String callsign;
String passcode;
float  latitude;
float  longitude;
float  elevation;
String write_key;

// structure to hold sensor measurements & calculated values
struct
{
  float stationPressure;         // station pressure (hPa) (mb)
  float seaLevelPressure;        // calculated SLP (hPa)
  float celsius;                 // temperature (°C)
  float fahrenheit;              // calculated temperature (°F)
  float humidity;                // relative humidity (%)
  float lightLevel;              // light intensity (lux)
  float cellVoltage;             // volts
  long  wifiRSSI;                // signal strength (dBm)
} sensorData;

// The ESP8266 Real Time Clock memory is arranged into blocks of 4 bytes.
// The RTC data structure MUST be padded to a 4-byte multiple.
// Maximum 512 bytes available.
// https://arduino-esp8266.readthedocs.io/en/latest/libraries.html#esp-specific-apis
// Use fixed width types to avoid variations between devices, for example,
// int is two bytes in Arduino UNO and four bytes in ESP8266.
struct
{
  uint32_t  crc32;               // 4 bytes    4 total
  uint16_t  sequence;            // 2 byte,    6 total
  uint8_t   bme280Fail;          // 1 byte,    7 total
  uint8_t   bh1750Fail;          // 1 byte,    8 total
  uint8_t   lowVcell;            // 1 byte,    9 total
  uint8_t   lowRSSI;             // 1 byte,   10 total
  float     timeAwake;           // 4 bytes,  14 total
  float     adc_factor;          // 4 bytes,  18 total not used
  uint8_t   pad[2];              // 2 bytes,  20 total
} rtcData;

// *******************************************************
// ********* INSTANTIATE DEVICES *************************
// *******************************************************
Adafruit_BME280 bme;             // barometric pressure / temperature / humidity sensor
BH1750 bh1750;                   // light level sensor
WiFiClient client;               // ThingSpeak
WiFiManager myWFM;               // captive portal

// drd is reset by stop() function in code, not by timeout
// drd uses rtc memory so it must start after rtcData and OTA areas
const int DRD_ADDRESS = OTA_OFFSET + sizeof(rtcData); // OTA offset + rtcData size
DoubleResetDetector drd(10, DRD_ADDRESS);

// *******************************************************
// ******************** SETUP ****************************
// *******************************************************
void setup()
{
  Wire.begin();                  // required for BME280 library
  Serial.begin( 115200 );
  pinMode(LED_BUILTIN, OUTPUT);

  // erase FS config data - used ONLY for testing
  if ( RESET_WIFI )
  {
    LittleFS.format();
  }

  // read configuration from FS json
  DEBUG_PRINTLN("\nMounting File System");
  openFS();

  // ***********************************************
  // define & add user parameters to config web page
  // ***********************************************

  //  parameter name (json id, Prompt, user input variable, length)
  myWFM.setCustomHeadElement("<h1 style=\"color:red; text-decoration: underline\">D1M-WX1 Weather Station</h1>");

  WiFiManagerParameter custom_text_aprs("<p style=\"color:red; font-weight: bold; text-decoration: underline\">APRS parameters:</p>");
  myWFM.addParameter( &custom_text_aprs );

  WiFiManagerParameter custom_callsign("callsign", "Callsign-SSID", wm_callsign, 10);
  myWFM.addParameter( &custom_callsign );

  WiFiManagerParameter custom_passcode("passcode", "Passcode", wm_passcode, 10);
  myWFM.addParameter( &custom_passcode );

  WiFiManagerParameter custom_latitude("latitude", "Latitude (dec. degrees, negative South)", wm_latitude, 15);
  myWFM.addParameter( &custom_latitude );

  WiFiManagerParameter custom_longitude("longitude", "Longitude (dec. degrees, negative West)", wm_longitude, 15);
  myWFM.addParameter( &custom_longitude );

  WiFiManagerParameter custom_elevation("elevation", "Elevation (m)", wm_elevation, 10);
  myWFM.addParameter( &custom_elevation );

  WiFiManagerParameter custom_text_ts("<p style=\"color:red; font-weight: bold; text-decoration: underline\">ThingSpeak parameters:</p>");
  myWFM.addParameter( &custom_text_ts );

  WiFiManagerParameter custom_channel("channel", "ThingSpeak Channel ID", wm_channel, 10);
  myWFM.addParameter( &custom_channel );

  WiFiManagerParameter custom_write_key("write_key", "ThingSpeak Write Key", wm_write_key, 20);
  myWFM.addParameter( &custom_write_key );

  // WiFiManager Callback functions
  myWFM.setAPCallback( configModeCallback );
  myWFM.setSaveConfigCallback( saveConfigCallback );

  // reset settings - used for testing ONLY
  if ( RESET_WIFI )
  {
    myWFM.resetSettings();
  }

  myWFM.setTimeout( 240 );    // in seconds

  if ( drd.detectDoubleReset() )
  {
    DEBUG_PRINTLN("Double Reset Detected");
    digitalWrite(LED_BUILTIN, LOW);
    myWFM.startConfigPortal( AP_PORTAL_NAME );
  }
  else
  {
    DEBUG_PRINTLN("No Double Reset Detected");
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_PRINTLN("Attempt WiFi connection");
    myWFM.autoConnect( AP_PORTAL_NAME );
  }

  if ( WiFi.status() != WL_CONNECTED )
  {
    DEBUG_PRINTLN("Failed to connect - restarting");
    delay( 3000 );
    ESP.reset();
  }

  // if you get here you have connected to the WiFi

  // get the user's parameters from WiFiManager
  wm_callsign  = custom_callsign.getValue();
  wm_passcode  = custom_passcode.getValue();
  wm_latitude  = custom_latitude.getValue();
  wm_longitude = custom_longitude.getValue();
  wm_elevation = custom_elevation.getValue();
  wm_channel   = custom_channel.getValue();
  wm_write_key = custom_write_key.getValue();

  // cast char array as String
  callsign  = wm_callsign;
  passcode  = wm_passcode;
  write_key = wm_write_key;

  // cast char array as numeric
  latitude  = atof( wm_latitude );
  longitude = atof( wm_longitude );
  elevation = atof( wm_elevation );

  // save the custom parameters to FS
  if ( saveConfigFlag )
  {
    DEBUG_PRINTLN("Save new config");
    saveConfig();
    //    adcCalibrate();
    writeRTCmemory();
  }
  drd.stop();                    // prevent detection on reboot

  // *********** Weather Station Program *******************
  readRTCmemory();               // get APRS sequence number & sensor status
  rtcData.sequence++;            // increment sequence, rollover at 999
  if (rtcData.sequence > 999 )
  {
    rtcData.sequence = 0;
  }
  constrain( rtcData.timeAwake, 0, 25 );

  readSensors();                 // read data into sensorData struct
  printToSerialPort();           // display data on local serial monitor
  // periodically check for an OTA
  // Note: if an OTA update is performed, program will be reset
  if ( rtcData.sequence % ( OTA_SPAN / SLEEP_INTERVAL ) == 0 )
  {
    unitStatus += "OTA check @ seq. " + String(rtcData.sequence);
    checkOTAupdate();            // check for OTA update
  }
  postToAPRS();                  // send data to APRS-IS
  postToThingSpeak();            // send data to ThingSpeak
  writeRTCmemory();              // save sequence & sensor status
  enterSleep( SLEEP_INTERVAL );  // go to low power sleep mode
} //setup()

// *******************************************************
// ******************** LOOP *****************************
// *******************************************************
void loop()
{
  // everything is done in setup()
} // loop()

// *******************************************************
// ******************* readSensors ***********************
// *******************************************************
void readSensors()
{
  if ( bme.begin(0x76, &Wire) )    // device is OK
  {
    sensorData.stationPressure = bme.readPressure() / 100.0F; // convert Pascals to hectoPascals
    sensorData.celsius = bme.readTemperature();
    sensorData.humidity = bme.readHumidity();

    // calculate fahrenheit
    sensorData.fahrenheit = CtoF(sensorData.celsius);

    // calculate the Sea Level Pressure from the station pressure and temperature
    sensorData.seaLevelPressure = SPtoSLP( sensorData.celsius,
                                           sensorData.stationPressure, atof( wm_elevation ) );

    if ( rtcData.bme280Fail )             // device was failed
    {
      unitStatus += "BME280 cleared. ";
      rtcData.bme280Fail = false;          // now cleared
    }
  }
  else                                     // device is failed
  {
    if ( rtcData.bme280Fail == false )     // device was OK
    {
      rtcData.bme280Fail = true;
      unitStatus += "BME280 failed. ";   // now failed
    }
  }

  // initialize BH1750 light sensor
  if ( bh1750.begin(BH1750::ONE_TIME_HIGH_RES_MODE))  // device is ok
  {
    bh1750.configure(BH1750::ONE_TIME_HIGH_RES_MODE); // for next time
    // read light level in lux
    sensorData.lightLevel = bh1750.readLightLevel();
    if ( rtcData.bh1750Fail == true )     // device was failed
    {
      unitStatus += "BH1750 cleared. ";
      rtcData.bh1750Fail = false;         // now ok
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.bh1750Fail == false )    // device was good
    {
      unitStatus += "BH1750 failed. ";
      rtcData.bh1750Fail = true;          // now failed
    }
  }

  // read analog voltage from the Analog to Digital Converter
  // on D1 Mini this is 0 - 1023 for voltages 0 to 3.2V
  // the D1M-WX1 has an external resistor to extend the range to 5.0 Volts
  // a fudgeFactor corrects for voltage divider component variation
  // as measured by the user in the calbration step

  //  sensorData.cellVoltage = 5.0 * analogRead( A0 ) * rtcData.adc_factor / 1023.0;
  sensorData.cellVoltage = 5.0 * analogRead( A0 ) / 1023.0;
  if ( sensorData.cellVoltage > MIN_VCELL ) // unit is OK
  {
    if ( rtcData.lowVcell )                 // was failed
    {
      unitStatus += "Vcell cleared. ";
      rtcData.lowVcell = false;             // now cleared
    }
  }
  else                                      // unit is bad
  {
    if ( rtcData.lowVcell == false )        // was good
    {
      unitStatus += "Vcell low. ";
      rtcData.lowVcell = true;              // now failed
    }
  }

  // read WiFi Received Signal Strength Indicator (RSSI)
  sensorData.wifiRSSI = WiFi.RSSI();
  if ( sensorData.wifiRSSI > MIN_RSSI )   // device is OK
  {
    if ( rtcData.lowRSSI )        // device was failed
    {
      unitStatus += "RSSI cleared. ";
      rtcData.lowRSSI = false;            // now cleared
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.lowRSSI == false )       // device was good
    {
      unitStatus += "RSSI low. ";
      rtcData.lowRSSI = true;             // now failed
    }
  }
} // readSensors()

// RTC Memory Functions: The ESP8266 internal Real Time Clock has unused memory
// that remains active during the Deep Sleep mode. This sketch stores WiFi connection
// information in RTC memory to speed up connection time.
// *******************************************************
// ******************* Read RTC Memory *******************
// *******************************************************
bool readRTCmemory()
{
  // this moves all the data in one block to the rtcData struct
  bool rtcValid = false;
  // offset data 32 bytes to avoid OTA area
  if ( ESP.rtcUserMemoryRead( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) ) )
  {
    // Calculate the CRC of what we just read from RTC memory,
    // but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32(((uint8_t*)&rtcData ) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 )
    {
      rtcValid = true;
    }
  }
  return rtcValid;
} // readRTCmemory()

// *******************************************************
// ****************** Write RTC Memory *******************
// *******************************************************
void writeRTCmemory()
{
  // offset data 32 bytes to avoid OTA area
  rtcData.timeAwake  = ( millis() - startTime ) / 1000.0;  // total awake time in seconds
  //rtcData.adc_factor writen on config
  rtcData.crc32      = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  // this moves all the data in the rtcData struct to RTC
  ESP.rtcUserMemoryWrite( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) );
} // writeRTCmemory()

// *******************************************************
// ******************** Calculate CRC32 ******************
// *******************************************************
// Cribbed from Bakke. Originated by others.
uint32_t calculateCRC32( const uint8_t *data, size_t length )
{
  uint32_t crc = 0xffffffff;
  while ( length-- )
  {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 )
    {
      bool bit = crc & 0x80000000;
      if ( c & i )
      {
        bit = !bit;
      }
      crc <<= 1;
      if ( bit )
      {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
} // calculateCRC32()

// *******************************************************
// ******************** ADC Calibrate *******************
// *******************************************************
//void adcCalibrate()
//{
//  // calibrate the ADC if the ADC voltage is within
//  // +/- vBand of 4.2 volts
//  float adcVal = 5.0 * analogRead( ADC_PIN ) / 1023.0;
//  float vBand = 0.25;  // 6% of full charge voltage
//  if ( adcVal > ( 4.2 - vBand ) && adcVal < ( 4.2 + vBand ) )
//  {
//    rtcData.adc_factor = 4.2 / adcVal;
//  }
//  else
//  {
//    if ( rtcData.adc_factor < 0.90 || rtcData.adc_factor > 1.10 )
//    {
//      rtcData.adc_factor = 1.0;
//    }
//  }
//  DEBUG_PRINT("ADC Val: "); DEBUG_PRINTLN(adcVal);
//  DEBUG_PRINT("ADC Factor: "); DEBUG_PRINTLN(rtcData.adc_factor);
//} //

// *******************************************************
// **************** Check for OTA Updates ****************
// *******************************************************
void checkOTAupdate()
{
  const String FW_URL_BASE = "https://w4krl.com/fota/";
  const String FW_VERSION_EXT = ".version";
  const String FW_FILENAME_EXT = ".ino.d1_mini.bin";
  const String FW_PATH = FW_FILENAME + "/";
  const String FW_VERSION_URL = FW_URL_BASE + FW_PATH + FW_FILENAME + FW_VERSION_EXT;
  const String FW_IMAGE_URL = FW_URL_BASE + FW_PATH + FW_FILENAME + FW_FILENAME_EXT;

  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  client->setInsecure();  // doesn't need fingerprint!!!!

  HTTPClient https;

  if (https.begin(*client, FW_VERSION_URL))
  {
    // start connection and send HTTP header
    int httpCode = https.GET();
    Serial.printf("[HTTPS] GET code: %d\n", httpCode);
    if ( httpCode > 0 )
    {
      // HTTP header has been sent and Server response header has been handled
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      {
        String newFWVersion = https.getString();
        int newVersion = newFWVersion.toInt();
        DEBUG_PRINT(F("Using version: ")); DEBUG_PRINTLN(FW_VERSION);
        DEBUG_PRINT(F("Found version: ")); DEBUG_PRINTLN(newVersion);
        if ( newVersion > FW_VERSION )
        {
          unitStatus += " Update to version " + newFWVersion + ".";
          postToThingSpeak();
          writeRTCmemory();
          // PROGRAM WILL RESTART AFTER UPDATE
          ESPhttpUpdate.update( *client, FW_IMAGE_URL );  // must be *client
        }
        else
        {
          unitStatus += " OK. ";
        }
      }
    }
    else
    {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  }
  else
  {
    DEBUG_PRINT(F("[HTTPS] Unable to connect\n"));
  }
}

// *******************************************************
// ********** Post data to ThingSpeak ********************
// *******************************************************
void postToThingSpeak()
{
  // assemble and post the data
  if ( client.connect( THINGSPEAK_SERVER, 80 ))
  {
    DEBUG_PRINTLN("ThingSpeak Server connected to channel: " + String(wm_channel));
    // declare dataString as a String and initialize with the API_WRITE_KEY
    String dataString = write_key;
    // cocatenate each field onto the end of dataString
    dataString += "&field1=" + String( sensorData.celsius );
    dataString += "&field2=" + String( sensorData.humidity );
    dataString += "&field3=" + String( rtcData.timeAwake );
    dataString += "&field4=" + String( sensorData.seaLevelPressure );
    dataString += "&field5=" + String( sensorData.lightLevel );
    dataString += "&field6=" + String( sensorData.cellVoltage );
    dataString += "&field7=" + String( sensorData.wifiRSSI );
    dataString += "&field8=" + String( sensorData.fahrenheit );
    dataString += "&status=" + unitStatus;
    DEBUG_PRINTLN( dataString );   // show ThingSpeak payload on serial monitor

    // find the number of characters in dataString
    String dataStringLength = String(dataString.length());

    // post the data to ThingSpeak
    client.println("POST /update HTTP/1.1");
    client.println("Host: " + THINGSPEAK_SERVER);
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + write_key);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + dataStringLength);
    client.println("");
    client.print(dataString);

    DEBUG_PRINTLN("ThingSpeak data sent.");
  }
  client.stop();
} // postToThingSpeak()

// *******************************************************
// ************ Print data to the serial port ************
// *******************************************************
void printToSerialPort()
{
  // buffer to store formatted print string
  char buf[60];  // must be >1 longer than the printed string
  // copy sensor data into short variables for readability
  float tc = sensorData.celsius;
  float tf = sensorData.fahrenheit;
  float rh = sensorData.humidity;
  float sl = sensorData.seaLevelPressure;
  float si = HPAtoINHG( sl );
  float lx = sensorData.lightLevel;
  float vc = sensorData.cellVoltage;

  DEBUG_PRINTLN(F("-------------------------------------------------"));
  DEBUG_PRINT("Sequence: "); DEBUG_PRINTLN(rtcData.sequence);
  // header line
  DEBUG_PRINTLN(F("   °C    (°F)   RH%  SLP mb    (in)     Lux  Volt"));
  // data line
  sprintf(buf, "%5.1f (%5.1f) %5.1f  %6.1f (%5.2f)  %6.0f  %4.2f", tc, tf, rh, sl, si, lx, vc);
  DEBUG_PRINTLN( buf );
  DEBUG_PRINTLN(F("-------------------------------------------------"));
} // printToSerialPort()

// *******************************************************
// ***************** Enter Sleep Mode ********************
// *******************************************************
void enterSleep(long sleep)
{
  DEBUG_PRINT(F("Sleeping for "));
  DEBUG_PRINT( sleep );
  DEBUG_PRINTLN(F(" seconds."));
  delay( 2 );                       // delay to let things settle
  // WAKE_RF_DEFAULT wakes the ESP8266 with Wi-Fi enabled
  ESP.deepSleep(sleep * 1000000L, WAKE_RF_DEFAULT);
} // enterSleep()

// *******************************************************
// *********** WiFi Manager Functions ********************
// *******************************************************
void openFS()
{
  // LittleFS is a Flash File System
  // https://arduino-esp8266.readthedocs.io/en/latest/filesystem.html
  // from wifiManager example AutoConnectWithFSParameters
  // https://github.com/tzapu/WiFiManager/tree/master/examples/AutoConnectWithFSParameters
  if ( LittleFS.begin() )
  {
    if ( LittleFS.exists( CONFIG_FILENAME ) )
    {
      // file exists - read and copy to globals
      File configFile = LittleFS.open( CONFIG_FILENAME, "r" );
      if ( configFile )
      {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        const size_t capacity = JSON_OBJECT_SIZE( USER_PARAMS ) + 300;
        DynamicJsonDocument doc( capacity );

        deserializeJson( doc, buf.get() );

        wm_callsign  = doc["callsign"];
        wm_passcode  = doc["passcode"];
        wm_latitude  = doc["latitude"];
        wm_longitude = doc["longitude"];
        wm_elevation = doc["elevation"];
        wm_channel   = doc["channel"];
        wm_write_key = doc["write_key"];
      }
      else
      {
        DEBUG_PRINTLN(F("Failed to load json config"));
      }
      configFile.close();
    }
  }
  else
  {
    DEBUG_PRINTLN(F("Failed to mount FS"));
  }
  //end read
} // openFS()

void configModeCallback( WiFiManager * myWiFiManager )
{
  DEBUG_PRINTLN(F("Entered config mode"));
  DEBUG_PRINTLN( WiFi.softAPIP() );
  drd.stop();
} // configModeCallback()

// callback when there is need to save config
void saveConfigCallback()
{
  DEBUG_PRINTLN(F("Config save requested"));
  saveConfigFlag = true;
} // saveConfigCallback()


void saveConfig()
{
  // saves user parameters in LittleFS
  const size_t capacity = JSON_OBJECT_SIZE( USER_PARAMS );
  DynamicJsonDocument doc( capacity );
  // copy globals to json
  doc["callsign"]   = wm_callsign;
  doc["passcode"]   = wm_passcode;
  doc["latitude"]   = wm_latitude;
  doc["longitude"]  = wm_longitude;
  doc["elevation"]  = wm_elevation;
  doc["channel"]    = wm_channel;
  doc["write_key"]  = wm_write_key;

  File configFile = LittleFS.open( CONFIG_FILENAME, "w" );
  if ( !configFile )
  {
    DEBUG_PRINTLN(F("Failed to write config"));
  }
  serializeJsonPretty( doc, Serial );
  DEBUG_PRINTLN("");
  serializeJson( doc, configFile );
  configFile.close();
  //end save
} // saveConfig()

// *******************************************************
// **************** Post data to APRS-IS *****************
// *******************************************************
void postToAPRS()
{
  // See http://www.aprs-is.net/Connecting.aspx
  String dataString = "";
  if ( client.connect( APRS_SERVER, APRS_PORT ) )
  {
    DEBUG_PRINTLN(F("APRS connected."));
  }
  else
  {
    DEBUG_PRINTLN(F("APRS connection failed."));
  }
  if ( client.connected() )
  {
    String rcvLine = client.readStringUntil('\n');
    DEBUG_PRINT(F("Rcvd: ")); DEBUG_PRINTLN(rcvLine);
    if ( rcvLine.indexOf("full") > 0 )
    {
      DEBUG_PRINTLN(F("APRS port full. Retrying."));
      client.stop();  // disconnect from port
      delay(500);
      client.connect( APRS_SERVER, APRS_PORT );
    }
    // send APRS-IS logon info
    dataString = "user " + callsign + " pass " + passcode;
    dataString += " vers IoT-Kits " + APRS_SOFTWARE_VERS; // softwarevers
    client.println( dataString );       // send to APRS-IS
    DEBUG_PRINT(F("Send: "));
    DEBUG_PRINTLN( dataString );

    if ( APRSverified() == true )
    {
      DEBUG_PRINTLN("APRS Login ok.");
      APRSsendWeather();
      APRSsendTelemetry();
      if ( unitStatus != "" )
      {
        APRSsendMessage( unitStatus );
      }
      client.stop();                  // disconnect from APRS-IS server
      DEBUG_PRINTLN("APRS done.");
    }
    else
    {
      DEBUG_PRINTLN("APRS user not verified.");
      unitStatus += "APRS user not verified. ";
    } // if APRSverified
  } // if client.connected
} // postToAPRS()

// *******************************************************
// **************** Verify logon to APRS-IS **************
// *******************************************************
boolean APRSverified()
{
  boolean verified = false;
  unsigned long timeBegin = millis();
  while ( millis() - timeBegin < APRS_TIMEOUT )
  {
    String rcvLine = client.readStringUntil('\n');
    DEBUG_PRINT("Rcvd: ");
    DEBUG_PRINTLN(rcvLine);
    if ( rcvLine.indexOf("verified") > 0 )
    {
      if ( rcvLine.indexOf("unverified") == -1 )
      {
        // "unverified" not found so we are in
        verified = true;
        break;
      }
    }
    delay(100);
  }
  return verified;
} // APRSverified()

// *******************************************************
// ************** Send weather data to APRS-IS ***********
// *******************************************************
// page 65 http://www.aprs.org/doc/APRS101.PDF
// Using Complete Weather Report Format — with Lat/Long position, no Timestamp
void APRSsendWeather()
{
  String dataString = callsign;
  dataString += ">APRS,TCPIP*:";
  dataString += "!" + APRSlocation( latitude, longitude );
  dataString += "_";               // required symbol code
  //  dataString += ".../...g..."; // no wind data
  dataString += ".../...";         // no wind data
  dataString += "t" + APRSpadTempF( sensorData.fahrenheit );
  //  dataString += "p...P...";    // no rainfall data
  dataString += "h" + APRSpadHumid( sensorData.humidity );
  dataString += "b" + APRSpadBaro( sensorData.seaLevelPressure );
  dataString += APRS_DEVICE_NAME;
  dataString += " ";
  dataString += APRS_SOFTWARE_VERS;
  client.println( dataString );      // send to APRS-IS
  DEBUG_PRINTLN("Send: " + dataString );      // print to serial port
} // APRSsendWeather()

// send directed message no ACK
void APRSsendMessage(String msg)
{
  // APRS101.pdf pg71
  String dataString = callsign;
  dataString += ">APRS,TCPIP*:";
  dataString += ":" + APRSpadCall(callsign) + ":";
  dataString += msg;
  DEBUG_PRINTLN("Send: " + dataString);
  client.println( dataString );         // send to APRS-IS
}

// *******************************************************
// ************* Send telemetry data to APRS *************
// *******************************************************
// page 68 http://www.aprs.org/doc/APRS101.PDF
void APRSsendTelemetry()
{
  String dataString = callsign;
  // start telemetry messages
  dataString += ">APRS,TCPIP*:";
  dataString += "T#" + APRSpadSequence( rtcData.sequence );             // sequence number
  dataString += "," +  APRSpadVcell( sensorData.cellVoltage );          // channel A1
  dataString += "," +  APRSpadRSSI( sensorData.wifiRSSI );              // channel A2
  dataString += "," +  APRSpadLightIntensity( sensorData.lightLevel );  // channel A3
  dataString += "," +  APRSpadTimeAwake( rtcData.timeAwake );           // channel A4
  dataString += ",";                                                    // channel A5 (spare)
  dataString += ",";                                                    // move on to digital channels
  // On aprs.fi:
  //   0 causes bit sense channel name to appear in bold with dark gray background
  //   1 causes channel name to be regular weight on a light gray background
  if ( rtcData.bme280Fail )
  {
    dataString += "0";
  }
  else
  {
    dataString += "1";
  }
  if ( rtcData.bh1750Fail )
  {
    dataString += "0";
  }
  else
  {
    dataString += "1";
  }
  if ( rtcData.lowVcell )
  {
    dataString += "0";
  }
  else
  {
    dataString += "1";
  }
  if ( rtcData.lowRSSI )
  {
    dataString += "0";
  }
  else
  {
    dataString += "1";
  }
  dataString += "0000";                 // unused digital channels
  dataString += "," + APRS_PROJECT;     // Project Title 0 - 23 characters
  client.println( dataString );         // send to APRS-IS

  DEBUG_PRINT("Send: ");
  DEBUG_PRINTLN( dataString );          // print to serial port

  // send telemetry definitions every TELEM_SPAN cycles
  if ( rtcData.sequence % ( APRS_DEFINITION_SPAN / SLEEP_INTERVAL ) == 0 || saveConfigFlag )
  {
    APRSsendTelemetryDefinitions( callsign );
    String msg = "APRS telemetry definitions sent @ seq ";
    msg += rtcData.sequence;
    DEBUG_PRINTLN( msg );
    unitStatus += msg + " ";
  }
} // APRSsendTelemetry()

// *******************************************************
// *********** Send APRS telemetry definitions ***********
// *******************************************************
void APRSsendTelemetryDefinitions(String callsign)
{
  String APRStelemHeader = callsign + ">APRS,TCPIP*::" + APRSpadCall( callsign ) + ":";
  // PARM - parameters
  String dataString = APRStelemHeader;
  //             Parameter Name   channel (number of characters)
  //             ==============   ======= ======================
  //  dataString += "PARM.";
  //  dataString += "Vcell";            // A1 (1-7)
  //  dataString += ",RSSI";            // A2 (1-7)
  //  dataString += ",Light";           // A3 (1-6)
  //  dataString += ",Awake";           // A4 (1-6)
  //  dataString += ",";                // A5 (1-5) spare
  //  dataString += ",BME28";           // B1 (1-6)
  //  dataString += ",BH17";            // B2 (1-5)
  //  dataString += ",loV";             // B3 (1-4)
  //  dataString += ",loS";             // B4 (1-4)
  //  //dataString += ",pB5";           // B5 (1-4) spare
  //  //dataString += ",pB6";           // B6 (1-3) spare
  //  //dataString += ",pB7";           // B7 (1-3) spare
  //  //dataString += ",pB8";           // B8 (1-3) spare

  // these definitions are expanded
  // verified to work with APRS.fi June 2022
  // spaces are tolerated, max length unknown
  //             Parameter Name   channel (number of characters)
  //             ==============   ======= ======================
  dataString += "PARM.";
  dataString += "Vcell";            // A1
  dataString += ",RSSI";            // A2
  dataString += ",Light";           // A3
  dataString += ",Awake";           // A4
  dataString += ",";                // A5 spare
  dataString += ",BME280 Fail";     // B1
  dataString += ",BH1750 Fail";     // B2
  dataString += ",Vcell Low";       // B3
  dataString += ",Signal Low";      // B4

  //dataString += ",pB5";           // B5 (1-4) spare
  //dataString += ",pB6";           // B6 (1-3) spare
  //dataString += ",pB7";           // B7 (1-3) spare
  //dataString += ",pB8";           // B8 (1-3) spare

  client.println( dataString );       // send to APRS-IS
  DEBUG_PRINTLN("Send: " + dataString);       // print to serial port

  // UNIT - parameter units
  dataString = APRStelemHeader;
  dataString += "UNIT.";
  dataString += "Vdc";              // A1 (1-7)
  dataString += ",dBm";             // A2 (1-7)
  dataString += ",lx";             // A3 (1-6)
  dataString += ",secs";            // A4 (1-6)
  dataString += ",";                // A5 (1-5) spare
  dataString += ",FAIL";              // B1 (1-6) BME280
  dataString += ",FAIL";              // B2 (1-5) BH1750
  dataString += ",LOW";              // B3 (1-4) low Vcell
  dataString += ",WEAK";              // B4 (1-4) low RSSI
  //  dataString += ",uB5";         // B5 (1-4) spare
  //  dataString += ",uB6";         // B6 (1-3) spare
  //  dataString += ",uB7";         // B7 (1-3) spare
  //  dataString += ",uB8";         // B8 (1-3) spare
  client.println( dataString );       // send to APRS-IS
  DEBUG_PRINTLN("Send: " + dataString);       // print to serial port

  // EQNS - equations to convert analog data byte to parameter value
  // formula: A * X^2 + B * X + C
  dataString = APRStelemHeader;
  dataString += "EQNS.";
  dataString += "0,0.0025,2.5";     // A1 convert 0-999 to 2.5-4.9975 v
  dataString += ",0,-1,0";          // A2 convert |RSSI| to dBm
  dataString += ",0.1218,0,0";      // A3 convert 0-999 to 0-121,557 lux
  dataString += ",0,0.025,0";       // A4 convert 0-999 to 0-24.975 seconds
  //  dataString += ",0,0,0";       // A5 (spare)
  client.println( dataString );     // send to APRS-IS
  DEBUG_PRINTLN("Send: " + dataString); // print to serial port

  // BITS - bit sense and project identity
  dataString = APRStelemHeader;
  dataString += "BITS.";
  dataString += "00000000";        // bit sense 0 = true
  dataString += ",";
  dataString += APRS_PROJECT;      // 23 char max
  client.println( dataString );    // send to APRS-IS
  DEBUG_PRINTLN("Send: " + dataString); // print to serial port
} // APRSsendTelemetryDefinitions()

// *******************************************************
// *********** Format callsign for APRS telemetry ********
// *******************************************************
// max length with SSID is 9, for ex.: WA3YST-13
// this pads a short call with spaces to the right
String APRSpadCall(String callSign)
{
  int len = callSign.length();    // number of characters in callsign
  String dataString = callSign;   // initialize dataString with callsign
  for (int i = len; i < 9; i++)
  {
    dataString += " ";            // pad right with spaces
  }
  return dataString;
}  // APRSpadCall()

// *******************************************************
// *************** Format location for APRS **************
// *******************************************************
String APRSlocation(float lat, float lon)
{
  // NOTE: abs() and % DO NOT WORK WITH FLOATS!!!
  // convert decimal latitude & longitude to DDmm.mmN/DDDmm.mmW
  char latID[2] = "N";             // North/South marker
  char lonID[2] = "E";             // East/West marker
  if ( lat < 0 )
  {
    strcpy(latID, "S");
    lat = -lat;                    // negate to make positive
  }
  if ( lon < 0 )
  {
    strcpy(lonID, "W");
    lon = -lon;                    // negate to make positive
  }
  int lat_degrees = (int)lat;
  float lat_minutes = 60 * (lat - lat_degrees);
  int lon_degrees = (int)lon;
  float lon_minutes = 60 * (lon - lon_degrees);

  char buf[19];
  sprintf(buf, "%02d%05.2f%s/%03d%05.2f%s", lat_degrees, lat_minutes, latID, lon_degrees, lon_minutes, lonID);
  return buf;
} // APRSlocation()

// *******************************************************
// ****** Format temperature in Fahrenheit for APRS ******
// *******************************************************
String APRSpadTempF(float tempF)
{
  String dataString = "";
  int tf = tempF + 0.5;    // round to integer temperature
  if ( tf > 999 )
  {
    tf = 999; // limit is 999 F
  }
  if ( tf < -99 )
  {
    tf = -99;
  }
  char buf[4];
  sprintf( buf, "%03d", tf );
  return buf;
} // APRSpadTempF

// *******************************************************
// *************** Format humidity for APRS **************
// *******************************************************
String APRSpadHumid(float humid)
{
  //  String dataString = "";
  int rh = humid + 0.5;        // rounds up and converts to integer
  if ( rh > 99 )
  { // max APRS humidity is 99%
    rh = 0;                    // APRS Protocol says 00 = 100%
  }
  char buf[3];
  sprintf( buf, "%02d", rh );
  return buf;
} // APRSpadHumid()

// *******************************************************
// *********** Format LiPo cell voltage for APRS *********
// *******************************************************
String APRSpadVcell(float volts)
{
  if ( volts < 2.5 )
  {
    volts = 2.5;
  }
  int vcell = ( volts - 2.5 ) * 400;  // convert 2.5 - 4.9975 to 0 - 999
  char buf[4];
  sprintf( buf, "%03d", vcell );
  return buf;
} // APRSpadVcell()

// *******************************************************
// ************** Format light level for APRS ************
// *******************************************************
String APRSpadLightIntensity(float lightLevel)
{
  // compress lux data by taking square root
  // expand in APRS receiver by squaring
  // see EQNS. in APRSsendTelemetryDefinitions
  int CHANNEL_MAX = 999;    // max value of analog channel (not documented)
  int VALUE_MAX = 121577;   // max value of BH1750

  int light = CHANNEL_MAX * sqrt( lightLevel / VALUE_MAX );   // convert 0-121,557 to 0 - 999
  char buf[4];
  sprintf( buf, "%03d", light );
  return buf;
} // APRSpadLightIntensity()

// *******************************************************
// ************** Format WiFi RSSI for APRS **************
// *******************************************************
String APRSpadRSSI(long strength)
{
  // RSSI is always negative
  strength = abs( strength );             // change to positive value
  char buf[4];
  sprintf( buf, "%03d", strength );
  return buf;
} // APRSpadRSSI()

// *******************************************************
// ********* Format sea level pressure for APRS **********
// *******************************************************
String APRSpadBaro(float barom)
{
  // pad barometric pressure in 10ths of hPa to 5 digits
  unsigned int bar10 = barom * 10;    // convert to 10th of millibars
  if ( bar10 > 99999 )
  { // upper limit
    bar10 = 99999;
  }
  char buf[6];
  sprintf( buf, "%05d", bar10 );
  return buf;
} // APRSpadBaro()

// *******************************************************
// ************** Format time awake for APRS *************
// *******************************************************
String APRSpadTimeAwake(float awake)
{
  // range is 0 to 24.975 seconds
  // converted to 0 to 999
  // multiplier = 999 / 24.975 = 40

  if ( awake > 24.975 )
  {
    awake = 24.975;
  }
  int val = 40 * awake;
  char buf[4];
  sprintf( buf, "%03d", val );
  return buf;
} // APRSpadTimeAwake()

// *******************************************************
// ********* Pad Telemetry Sequence Number ***************
// *******************************************************
String APRSpadSequence(uint16_t sequence)
{
  // pad the sequence number to three digits
  char buf[4];
  sprintf(buf, "%03d", sequence);
  return buf;
} // APRSpadSequence()

// *******************************************************
// ************** UNIT CONVERSIONS ***********************
// *******************************************************

float HPAtoINHG(float hpa)
{
  return 0.0295299830714 * hpa;  // hPa (mb) to inHg pressure
}

float CtoF(float C)
{
  return 1.8 * C + 32.0;  // Celcius to Fahrenheit
}

// *******************************************************
// Calculate relative sea level pressure from absolute station pressure in hPa
// temperature in °C, elevation in m
// http://www.sandhurstweather.org.uk/barometric.pdf
// http://keisan.casio.com/exec/system/1224575267
// *******************************************************
float SPtoSLP(float celsius, float stationPressure, float elevation)
{
  return stationPressure / pow(2.718281828, -(elevation / ((273.15 + celsius) * 29.263)));
} // SPtoSLP()

// *******************************************************
// *********************** END ***************************
// *******************************************************
