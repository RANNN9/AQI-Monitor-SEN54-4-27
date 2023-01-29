/*
  Project Name:   PM2.5 monitor
  Description:    Regularly sample and log PM 2.5 levels

  See README.md for target information and revision history
*/

#include <SensirionI2CSen5x.h>
#include <Wire.h>

#include "config.h"
#include "secrets.h"

#ifdef WIFI
  // ESP32 WiFi
  #include <WiFi.h>

  // ESP8266 WiFi
  // #include <ESP8266WiFi.h>

  // Use WiFiClient class to create TCP connections and talk to hosts
  WiFiClient client;
#endif

//Create SEN5x sensor instance
SensirionI2CSen5x sen5x;

// global variables

// PM sensor data
typedef struct
{
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm10p0;
  float massConcentrationPm4p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;  // Not supported by SEN54 (only SEN55)
} envData;
envData sensorData;

float humidityTotal = 0;  // running total of humidity over report interval
float tempFTotal = 0;     // running total of temperature over report interval
float vocTotal = 0;       // running total of VOC over report interval
float avgTempF = 0;       // average temperature over report interval         
float avgHumidity = 0;    // average humidity over report interval
float avgVOC = 0;         // average VOC over report interval

float pm25Total = 0;      // running total of humidity over report interval
float avgPM25;            // average PM2.5 over report interval

unsigned long prevReportMs = 0;   // Timestamp for measuring elapsed capture time
unsigned long prevSampleMs  = 0;  // Timestamp for measuring elapsed sample time
unsigned int numSamples = 0;      // Number of overall sensor readings over reporting interval
unsigned int numReports = 0;      // Number of capture intervals observed
// used by thingspeak and dweet
float MinPm25 = 1999;   /* Observed minimum PM2.5 */
float MaxPm25 = -99;   /* Observed maximum PM2.5 */

bool internetAvailable;
int rssi;

// External function dependencies
#ifdef DWEET
  extern void post_dweet(float pm25, float minaqi, float maxaqi, float aqi, float tempF, float vocIndex, float humidity);
#endif

#ifdef THINGSPEAK
  post_thingspeak();
#endif

#ifdef INFLUX
  extern void post_influx(float pm25, float aqi, float tempF, float vocIndex, float humidity);
#endif

#ifdef MQTT
  // MQTT interface depends on the underlying network client object, which is defined and
  // managed here (so needs to be defined here).
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client pm25_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

  // Adafruit PMSA003I
  extern int mqttDeviceWiFiUpdate(int rssi);
  extern int mqttSensorUpdate(float pm25, float aqi);
#endif

void setup() 
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);

    // Confirm key site configuration parameters
    debugMessage("PM2.5 monitor started");
    debugMessage("Client ID: " + String(CLIENT_ID));
    debugMessage(String(SAMPLE_INTERVAL) + " second sample interval");
    debugMessage(String(REPORT_INTERVAL) + " minute report interval");
  #endif

  initSensor();
  // Remember current clock time
  prevReportMs = prevSampleMs = millis();
}

void loop()
{
  // update current timer value
  unsigned long currentMillis = millis();

  // is it time to read the sensor
  if((currentMillis - prevSampleMs) >= (SAMPLE_INTERVAL * 1000)) // converting SAMPLE_INTERVAL into milliseconds
  {
    if (!readSensor())
    {
      // handle error condition
    }
    numSamples++;

    // convert temp from C to F
    sensorData.ambientTemperature = 32.0 + (1.8*sensorData.ambientTemperature);
    tempFTotal += sensorData.Temperature;
    humidityTotal += sensorData.ambientHumidity;
    vocTotal += sensorData.vocIndex;

    // add to the running totals
    pm25Total += sensorData.massConcentrationPm2p5;

    debugMessage(String("PM2.5 reading is ") + sensorData.massConcentrationPm2p5 + " or AQI " + pm25toAQI(sensorData.massConcentrationPm2p5));
    debugMessage(String("Temp is ") + sensorData.ambientTemperature + " F");
    debugMessage(String("Humidity is ") + sensorData.ambientHumidity + "%");
    debugMessage(String("VOC level is ") + sensorData.VocIndex);

    // Save sample time
    prevSampleMs = currentMillis;
  }

  // is it time to report averaged values
  if((currentMillis - prevReportMs) >= (REPORT_INTERVAL * 60 *1000)) // converting REPORT_INTERVAL into milliseconds
  {
    #ifdef WIFI
      if(networkBegin())
      {
        internetAvailable = true;
      }
    #endif
    if (numSamples != 0) 
    {
      avgPM25 = pm25Total / numSamples;
      if(avgPM25 > MaxPm25) MaxPm25 = avgPM25;
      if(avgPM25 < MinPm25) MinPm25 = avgPM25;
      debugMessage(String("Average PM2.5 reading for this ") + REPORT_INTERVAL + " minute report interval  is " + avgPM25 + " or AQI " + pm25toAQI(avgPM25));

      avgTempF = tempFTotal / numSamples;
      avgVOC = vocTotal / numSamples;
      avgHumidity = humidityTotal / numSamples;
  
      /* Post both the current readings and historical max/min readings to the web */
      #ifdef DWEET
        post_dweet(avgPM25,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),pm25toAQI(avgPM25),avgTempF,avgVOC,avgHumidity);
      #endif
  
      // Also post the AQI sensor data to ThingSpeak
      #ifdef THINGSPEAK
        post_thingspeak(avgPM25,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),pm25toAQI(avgPM25));
      #endif

      // And store data to Influx DB    
      #ifdef INFLUX
        post_influx(avgPM25,pm25toAQI(avgPM25),avgTempF,avgVOC,avgHumidity);
      #endif

      #ifdef MQTT
        mqttDeviceWiFiUpdate(rssi);
        mqttSensorUpdate(avgPM25, pm25toAQI(avgPM25));
      #endif

      // Reset counters and accumulators
      prevReportMs = currentMillis;
      numSamples = 0;
      pm25Total = 0;
      // Sparkfun SEN5x
      // tempFTotal = 0;
      // vocTotal = 0;
      // humidityTotal = 0;
    }
  }
}

#ifdef WIFI
  bool networkBegin()
  {
    // set hostname has to come before WiFi.begin
    WiFi.hostname(CLIENT_ID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    // Attempts WiFi connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        rssi = abs(WiFi.RSSI());
        debugMessage(String("WiFi IP address issued from ") + WIFI_SSID + " is " + WiFi.localIP().toString());
        debugMessage(String("WiFi RSSI is: ") + rssi + " dBm");
        return true;
      }
      debugMessage(String("Connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " to " + WIFI_SSID + " failed, trying again in " + CONNECT_ATTEMPT_INTERVAL + " seconds");
      // use of delay() OK as this is initialization code
      delay(CONNECT_ATTEMPT_INTERVAL * 1000); // convered into milliseconds
    }
    debugMessage("Failed to connect to WiFi");
    return false;
  }
#endif

int initSensor()
{
  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];

  // Handle two ESP32 I2C ports
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    Wire1.begin();
    sen5x.begin(Wire1);
  #else
    Wire.begin();
    sen5x.begin(Wire);
  #endif
  
  error = sen5x.deviceReset();
  if (error) 
  {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " error during SEN5x reset");
    return 0;
  }
  
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
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error setting temp offset");
  } else {
      debugMessage(String("Temperature Offset set to ") + tempOffset + " degrees C");
  }

  // Start Measurement
  error = sen5x.startMeasurement();
  if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error during SEN5x startMeasurement");
      return 0;
  }
  debugMessage("SEN5x initialized");
  return 1;
}

int readSensor()
{
  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];
  error = sen5x.readMeasuredValues(
        sensorData.massConcentrationPm1p0, sensorData.massConcentrationPm2p5, sensorData.massConcentrationPm4p0,
        sensorData.massConcentrationPm10p0, sensorData.ambientHumidity, sensorData.ambientTemperature, sensorData.vocIndex,
        sensorData.noxIndex);
  if (error) 
  {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error during SEN5x read");
      prevSampleMs = currentMillis;
      return 0;
  }
  // successful read
  debugMessage("Successful sensor read");
  return 1;
}

float pm25toAQI(float pm25)
// Converts pm25 reading to AQI using the AQI Equation
// (https://forum.airnowtech.org/t/the-aqi-equation/169)
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

void debugMessage(String messageText)
// wraps Serial.println as #define conditional
{
  #ifdef DEBUG
    Serial.println(messageText);
    Serial.flush();  // Make sure the message gets output (before any sleeping...)
  #endif
}