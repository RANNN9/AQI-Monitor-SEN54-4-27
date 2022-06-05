/*
 * Air Quality Monitor built using an Adafruit Feather Huzzah with a Sensirion SEN54 sensor.
 * Uses Sensirion's Arduino library for the SEN5x series. 
 * 
 * Reports observations over Wifi via the Feather Huzzah.  Data is posted to Dweet.io and
 * ThingSpeak, and stored via an InfluxDB service.  Because the data gets 
 * reported to the web there's no display, though useful information is output via the serial
 * monitor to streamline development.
 *
 * The SEN54 reports particulate mass concentrations (microgram per cubic meter) for 1, 2.5,
 * 4, and 10 micron particulates as well as VOC plus temperature and humidity.  Communication is
 * via I2C.
 * 
 * Note that Sensirion requires the following declaration:
 */
 /*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <SensirionI2CSen5x.h>
#include <Wire.h>  
#include <ESP8266WiFi.h>
#include "secrets.h"

// The used commands use up to 48 bytes. On some Arduinos the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

//Create Instance of SEN5x sensor
SensirionI2CSen5x sen5x;

// Use WiFiClient class to create TCP connections and talk to hosts
WiFiClient client;

/*
 * Need to accumulate several pieces of data to calculate ongoing readings
 *
 * In a moving vehicle good values are a sample delay of 2 seconds and a capture
 * delay (reporting inverval) of 2 minutes.  At a stationary location such as a home
 * weather station good values are a sample delay of 5 seconds and a capture delay
 * of 5 minutes.
 *
 */
const int sampleDelay = 5;         /* Interval at which temperature sensor is read (in seconds) */
const int reportDelay = 15;          /* Interval at which samples are averaged & reported (in minutes) */
const unsigned long reportDelayMs = reportDelay * 60 * 1000L;  /* Calculation interval */
const unsigned long sampleDelayMs = sampleDelay * 1000L;       /* Sample delay interval */
unsigned long prevReportMs = 0;  /* Timestamp for measuring elapsed capture time */
unsigned long prevSampleMs  = 0;  /* Timestamp for measuring elapsed sample time */
unsigned int numSamples = 0;  /* Number of overall sensor readings over reporting interval */
unsigned int numReports = 0; /* Number of capture intervals observed */

float Pm25 = 0;        /* Air Quality reading (PM2.5) over capture interval */
float CurrentPm25 = 0; /* Current PM2.5 reading from AQI sensor */
float AvgPm25;         /* Average PM2.5 as reported last capture interval */
float MinPm25 = 1999;   /* Observed minimum PM2.5 */
float MaxPm25 = -99;   /* Observed maximum PM2.5 */

float TemperatureF = 0;
float CurrentTempF = 0;
float AvgTempF = 0;

float Humidity = 0;
float CurrentHumidity = 0;
float AvgHumidity = 0;

float VocIndex = 0;
float CurrentVoc = 0;
float AvgVoc = 0;

extern void post_influx(float pm25, float aqi, float tempF, float vocIndex, float humidity);


void setup() {
  
  Serial.begin(115200);

  Serial.print("Sampling delay: ");  Serial.print(sampleDelay); Serial.println(" seconds");
  Serial.print("Reporting delay: "); Serial.print(reportDelay); Serial.println(" minutes");


  // We start by connecting to a WiFi network           
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Give some feedback while we wait...
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
     
  Serial.println("done");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Sampling...please wait");
  
  // Wait ten seconds for sensor to boot up!
  delay(10000);


  Wire.begin();
  sen5x.begin(Wire);
  
  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  if (error) {
      Serial.print("Error trying to execute deviceReset(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
#endif
  
  // set a temperature offset in degrees celsius
  // By default, the temperature and humidity outputs from the sensor
  // are compensated for the modules self-heating. If the module is
  // designed into a device, the temperature compensation might need
  // to be adapted to incorporate the change in thermal coupling and
  // self-heating of other device components.
  //
  // A guide to achieve optimal performance, including references
  // to mechanical design-in examples can be found in the app note
  // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
  // Please refer to those application notes for further information
  // on the advanced compensation settings used
  // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and
  // `setRhtAccelerationMode`.
  //
  // Adjust tempOffset to account for additional temperature offsets
  // exceeding the SEN module's self heating.
  float tempOffset = 0.0;
  error = sen5x.setTemperatureOffsetSimple(tempOffset);
  if (error) {
      Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      Serial.print("Temperature Offset set to ");
      Serial.print(tempOffset);
      Serial.println(" deg. Celsius");
  }

  // Start Measurement
  error = sen5x.startMeasurement();
  if (error) {
      Serial.print("Error trying to execute startMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  Serial.println("AQI sensor found!");

  // Remember current clock time
  prevReportMs = prevSampleMs = millis();
}

void  loop() {
  uint16_t error;
  char errorMessage[256];
  // Read Measurement
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;  // Not supported by SEN54 (only SEN55)
  
  unsigned long currentMillis = millis();    // Check current timer value
  
  /* See if it is time to read the sensors */
  if( (currentMillis - prevSampleMs) >= sampleDelayMs) {
    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        prevSampleMs = currentMillis;
    } 
    else {
      // Since we have a sample, count it
      numSamples ++;  // Since we started at 0...
  
      CurrentPm25 = massConcentrationPm2p5;
      Pm25 +=  CurrentPm25;

      CurrentTempF = 32.0 + (1.8*ambientTemperature);
      TemperatureF += CurrentTempF;

      CurrentVoc = vocIndex;
      VocIndex += CurrentVoc;

      CurrentHumidity = ambientHumidity;
      Humidity += CurrentHumidity;
  
      Serial.print("Read data PM25: ");
      Serial.print(CurrentPm25);
      Serial.print(" = AQI: ");
      Serial.print(pm25toAQI(CurrentPm25));
      Serial.print(", VOC: ");
      Serial.print(CurrentVoc);
      Serial.print(" (@ ");
      Serial.print(CurrentTempF);
      Serial.print("F, ");
      Serial.print(CurrentHumidity);
      Serial.println("%)");

      
      // Save sample time
      prevSampleMs = currentMillis;
    }
  }
  /* Now check and see if it is time to report averaged values */
  if( (currentMillis - prevReportMs) >= reportDelayMs) {

    if(numSamples == 0 ) {
      // Never got any data from the sensor, so need to flag that by dweeting a pm25 of -1 and AQI of 0
      // Don't bother reporting data to ThingSpeak though (we don't have any!)
      // (Might still have reasonable max/min values)
      post_dweet(-1.0,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),0.0,1000.0,-1.0,-1.0);

      // Reset counters and accumulators
      prevReportMs = currentMillis;
      numSamples = 0;
      Pm25 = 0;
      TemperatureF = 0;
      VocIndex = 0;
      Humidity = 0;
    }
    else {
      AvgPm25 = Pm25 / numSamples;
      if(AvgPm25 > MaxPm25) MaxPm25 = AvgPm25;
      if(AvgPm25 < MinPm25) MinPm25 = AvgPm25;

      AvgTempF = TemperatureF / numSamples;

      AvgVoc = VocIndex / numSamples;

      AvgHumidity = Humidity / numSamples;
  
      /* Post both the current readings and historical max/min readings to the web */
      post_dweet(AvgPm25,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),pm25toAQI(AvgPm25),AvgTempF,AvgVoc,AvgHumidity);
  
      // Also post the AQIQ sensor data to ThingSpeak
      post_thingspeak(AvgPm25,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),pm25toAQI(AvgPm25));

      // And store data to Influx DB (via remote server)    
      post_influx(AvgPm25,pm25toAQI(AvgPm25),AvgTempF,AvgVoc,AvgHumidity);

      
      // Reset counters and accumulators
      prevReportMs = currentMillis;
      numSamples = 0;
      Pm25 = 0;
      TemperatureF = 0;
      VocIndex = 0;
      Humidity = 0;
    }
  }
}

// Post a dweet to report sensor data readings.  This routine blocks while
// talking to the network, so may take a while to execute.
const char* dweet_host = "dweet.io";
void post_dweet(float pm25, float minaqi, float maxaqi, float aqi, float tempF, float vocIndex, float humidity)
{
  
  if(WiFi.status() != WL_CONNECTED) {
    Serial.print("Lost network connection to '");
    Serial.print(WIFI_SSID);
    Serial.println("'!");
    return;
  }
  
  Serial.print("connecting to ");
  Serial.print(dweet_host);
  Serial.print(" as ");
  Serial.println(String(DWEET_DEVICE));
      
  // Use our WiFiClient to connect to dweet
  if (!client.connect(dweet_host, 80)) {
    Serial.println("connection failed");
    return;
  }

  long rssi = WiFi.RSSI();
  IPAddress ip = WiFi.localIP();

  // Use HTTP post and send a data payload as JSON
  
  String postdata = "{\"wifi_rssi\":\""     + String(rssi)           + "\"," +
                     "\"AQI\":\""           + String(aqi,2)          + "\"," +
                     "\"address\":\""       + ip.toString()          + "\"," +
                     "\"temperature\":\""   + String(tempF,1)        + "\"," +
                     "\"vocIndex\":\""      + String(vocIndex,1)     + "\"," +
                     "\"humidity\":\""      + String(humidity,1)     + "\"," +
                     "\"PM25_value\":\""    + String(pm25,2)         + "\"," +
                     "\"min_AQI\":\""       + String(minaqi,2)       + "\"," + 
                     "\"max_AQI\":\""       + String(maxaqi,2)       + "\"}";
  // Note that the dweet device 'name' gets set here, is needed to fetch values
  client.println("POST /dweet/for/" + String(DWEET_DEVICE) + " HTTP/1.1");
  client.println("Host: dweet.io");
  client.println("User-Agent: ESP8266 (orangemoose)/1.0");
  client.println("Cache-Control: no-cache");
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(postdata.length());
  client.println();
  client.println(postdata);
  Serial.println(postdata);

  delay(1500);  
  // Read all the lines of the reply from server (if any) and print them to Serial Monitor
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
   
  Serial.println("closing connection");
  Serial.println();
}

// ThingSpeak data upload
const char* ts_server = "api.thingspeak.com";
void post_thingspeak(float pm25, float minaqi, float maxaqi, float aqi) {
  if (client.connect(ts_server, 80)) {
    
    // Measure Signal Strength (RSSI) of Wi-Fi connection
    long rssi = WiFi.RSSI();
    Serial.print("RSSI: ");
    Serial.println(rssi);

    // Construct API request body
    String body = "field1=";
           body += String(aqi,2);
           body += "&";
           body += "field2=";
           body += String(pm25,2);
           body += "&";
           body += "field3=";
           body += String(maxaqi,2);
           body += "&";
           body += "field4=";
           body += String(minaqi,2);


    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("User-Agent: ESP8266 (orangemoose)/1.0");
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + String(THINGS_APIKEY));
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + String(body.length()));
    client.println("");
    client.print(body);
    Serial.println(body);

  }
  delay(1500);
    
  // Read all the lines of the reply from server (if any) and print them to Serial Monitor
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  client.stop();
}

// Converts pm25 reading to AQI using the AQI Equation
// (https://forum.airnowtech.org/t/the-aqi-equation/169)

float pm25toAQI(float pm25)
{  
  if(pm25 <= 12.0)       return(fmap(pm25,  0.0, 12.0,  0.0, 50.0));
  else if(pm25 <= 35.4)  return(fmap(pm25, 12.1, 35.4, 51.0,100.0));
  else if(pm25 <= 55.4)  return(fmap(pm25, 35.5, 55.4,101.0,150.0));
  else if(pm25 <= 150.4) return(fmap(pm25, 55.5,150.4,151.0,200.0));
  else if(pm25 <= 250.4) return(fmap(pm25,150.5,250.4,201.0,300.0));
  else if(pm25 <= 500.4) return(fmap(pm25,250.5,500.4,301.0,500.0));
  else return(505.0);  // AQI above 500 not recognized
}

float fmap(float x, float xmin, float xmax, float ymin, float ymax)
{
    return( ymin + ((x - xmin)*(ymax-ymin)/(xmax - xmin)));
}

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}
