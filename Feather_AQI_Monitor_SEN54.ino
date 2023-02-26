/*
  Project Name:   PM2.5 monitor
  Description:    Regularly sample and log PM 2.5 levels

  See README.md for target information and revision history
*/

#include <SensirionI2CSen5x.h>
#include <Wire.h>

#include "config.h"
#include "secrets.h"

// Use WiFiClient class to create TCP connections and talk to hosts
WiFiClient client;

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

bool internetAvailable = false;
int rssi;

// External function dependencies
#ifdef DWEET
  extern void post_dweet(float pm25, float minaqi, float maxaqi, float aqi, float tempF, float vocIndex, float humidity);
#endif

#ifdef THINGSPEAK
  extern void post_thingspeak(float pm25, float minaqi, float maxaqi, float aqi);
#endif

#ifdef INFLUX
  extern bool post_influx(float pm25, float aqi, float tempF, float vocIndex, float humidity, int rssi);
#endif

#ifdef MQTT
  // MQTT interface depends on the underlying network client object, which is defined and
  // managed here (so needs to be defined here).
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client pm25_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

  // Adafruit PMSA003I
  extern bool mqttDeviceWiFiUpdate(int rssi);
  extern bool mqttSensorUpdate(float pm25, float aqi, float tempF, float vocIndex, float humidity);
#endif

void setup() 
{
  // handle Serial first so status messages and debugMessage() work
  Serial.begin(115200);
  // wait for serial port connection
  // while (!Serial);  // FIX: Infinite loop if no serial connection
  delay(5000);   // Workaround to give Serial port a chance to open
  
  // Enable LED for visual confirmation of reporting (and turn it off)
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_OFF);    // turn the LED

  // Display key configuration parameters
  Serial.println("PM2.5 monitor started");
  Serial.println(String("Sample interval is ") + SAMPLE_INTERVAL + " seconds");
  Serial.println(String("Report interval is ") + REPORT_INTERVAL + " minutes");
  Serial.println(String("Internet service reconnect delay is ") + CONNECT_ATTEMPT_INTERVAL + " seconds");

  // handle error condition
  initSensor();

  // Remember current clock time
  prevReportMs = prevSampleMs = millis();

  if(networkConnect()) {
    internetAvailable = true;
  }

  debugMessage("----- Sampling -----");
}

void loop()
{
  // update current timer value
  unsigned long currentMillis = millis();

  // is it time to read the sensor
  if((currentMillis - prevSampleMs) >= (SAMPLE_INTERVAL * 1000)) // converting SAMPLE_INTERVAL into milliseconds
  {
    if (readSensor()){
      numSamples++;

      // convert temp from C to F
      sensorData.ambientTemperature = 32.0 + (1.8*sensorData.ambientTemperature);

      // add to the running totals
      pm25Total += sensorData.massConcentrationPm2p5;
      tempFTotal += sensorData.ambientTemperature;
      humidityTotal += sensorData.ambientHumidity;
      vocTotal += sensorData.vocIndex;

      Serial.print("Current Readings: ");
      Serial.print(String("PM2.5: ") + sensorData.massConcentrationPm2p5 + " = AQI " + pm25toAQI(sensorData.massConcentrationPm2p5));
      Serial.print(String(", Temp: ") + sensorData.ambientTemperature + " F");
      Serial.print(String(", Humidity:  ") + sensorData.ambientHumidity + "%");
      Serial.println(String(", VOC: ") + sensorData.vocIndex);
      debugText(String("Sample #") + numSamples + String(", running totals: "),false);
      debugText(String("PM25 total: ") + pm25Total,false);
      debugText(String(", TempF total: ") + tempFTotal,false);
      debugText(String(", Humidity total: ") + humidityTotal,false);
      debugText(String(", VOC total: ") + vocTotal,true);

      // Save sample time
      prevSampleMs = currentMillis;
    }
    else
    {
      debugMessage("Could not read SEN54 sensor data");
    }
  }

  // is it time to report averaged values
  if((currentMillis - prevReportMs) >= (REPORT_INTERVAL * 60 *1000)) // converting REPORT_INTERVAL into milliseconds
  {
    if (numSamples != 0) 
    {
      avgPM25 = pm25Total / numSamples;
      if(avgPM25 > MaxPm25) MaxPm25 = avgPM25;
      if(avgPM25 < MinPm25) MinPm25 = avgPM25;

      avgTempF = tempFTotal / numSamples;
      avgVOC = vocTotal / numSamples;
      avgHumidity = humidityTotal / numSamples;

      Serial.println("----- Reporting -----");
      Serial.print(String("Reporting averages (") + REPORT_INTERVAL + String(" minute): "));

      Serial.print(String("PM2.5: ") + avgPM25 + String(" = AQI ") + pm25toAQI(avgPM25));
      Serial.print(String(", Temp: ") + avgTempF + String(" F"));
      Serial.print(String(", Humidity: ") + avgHumidity + String("%"));
      Serial.println(String(", VoC: ") + avgVOC);

      // reconnect to WiFi if needed
      if (WiFi.status() != WL_CONNECTED){
          WiFi.disconnect();
          WiFi.reconnect();
      }

      // Visual indication we're updating remote/internet services
      blink_led();

      /* Post both the current readings and historical max/min readings to the internet */
      #ifdef DWEET
        post_dweet(avgPM25,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),pm25toAQI(avgPM25),avgTempF,avgVOC,avgHumidity);
      #endif
  
      // Also post the AQI sensor data to ThingSpeak
      #ifdef THINGSPEAK
        post_thingspeak(avgPM25,pm25toAQI(MinPm25),pm25toAQI(MaxPm25),pm25toAQI(avgPM25));
      #endif

      #ifdef INFLUX
        if(!post_influx(avgPM25,pm25toAQI(avgPM25),avgTempF,avgVOC,avgHumidity,rssi))
          debugMessage("Did not write to influxDB");
      #endif

      #ifdef MQTT
        if(!mqttDeviceWiFiUpdate(rssi))
          debugMessage("Did not write device data to MQTT broker");
        if(!mqttSensorUpdate(avgPM25, pm25toAQI(avgPM25),avgTempF,avgVOC,avgHumidity))
          debugMessage("Did not write environment data to MQTT broker");
      #endif

      debugMessage("----- Sampling -----");
      // Reset counters and accumulators
      prevReportMs = currentMillis;
      numSamples = 0;
      pm25Total = 0;
      tempFTotal = 0;
      vocTotal = 0;
      humidityTotal = 0;
    }
    else
    {
      debugMessage("No samples to average and report on");
    }
  }
}

bool networkConnect()
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
      debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString());
      debugMessage(String("WiFi RSSI is: ") + rssi + " dBm");
      return true;
    }
    debugMessage(String("Connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " to " + WIFI_SSID + " failed");
    // use of delay() OK as this is initialization code
    delay(CONNECT_ATTEMPT_INTERVAL * 1000); // convered into milliseconds
  }
  return false;
}

bool initSensor()
{
  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];

  // If we're simulating the sensor there's nothing to init & we're good to go
  #ifdef SIMULATE_SENSOR
    return true;
  #endif

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
    return false;
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
      return false;
  }
  debugMessage("SEN5x initialized");
  return true;
}

bool readSensor()
{
  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];

  // If we're simulating the sensor generate some reasonable values for measurements
  #ifdef SIMULATE_SENSOR
    sensorData.massConcentrationPm1p0 = random(0,360)/10.0;
    sensorData.massConcentrationPm2p5 = random(0,360)/10.0;
    sensorData.massConcentrationPm4p0 = random(0,720)/10.0;
    sensorData.massConcentrationPm10p0 = random(0,1550)/10.0;
    sensorData.ambientHumidity = 30.0 + (random(0,250)/10.0);
    sensorData.ambientTemperature = 15.0 + (random(0,101)/10.0);
    sensorData.vocIndex = random(0,3500)/10.0;
    sensorData.noxIndex = random(0,2500)/10.0;
    return true;
  #endif

  error = sen5x.readMeasuredValues(
        sensorData.massConcentrationPm1p0, sensorData.massConcentrationPm2p5, sensorData.massConcentrationPm4p0,
        sensorData.massConcentrationPm10p0, sensorData.ambientHumidity, sensorData.ambientTemperature, sensorData.vocIndex,
        sensorData.noxIndex);
  if (error) 
  {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " error during SEN5x read");
      return false;
  }
  return true;
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

// Output debug text to Serial, allows optional newline after the text
void debugText(String outputText,bool newline)
{
  #ifdef DEBUG
    if(newline) {
      Serial.println(outputText);
      Serial.flush();
    }
    else {
      Serial.print(outputText);
    }
  #endif
}

void blink_led()
{
      // Blink the LED
      digitalWrite(LED_BUILTIN, LED_BUILTIN_ON);   // turn the LED on
      delay(500);                // wait for a half second
      digitalWrite(LED_BUILTIN, LED_BUILTIN_OFF);    // turn the LED
      delay(500);
}