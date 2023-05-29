/*
  Project Name:   PM2.5 monitor
  Description:    Regularly sample and log PM 2.5 levels

  See README.md for target information and revision history
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// initialize pm25 sensor
#include <SensirionI2CSen5x.h>
SensirionI2CSen5x sen5x;

// activate only if using network data endpoints
#if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif
#endif

#ifdef SCREEN
#include <Adafruit_ThinkInk.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

// 2.96" greyscale display with 196x128 pixels
// colors are EPD_WHITE, EPD_BLACK, EPD_RED, EPD_GRAY, EPD_LIGHT, EPD_DARK
ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// screen layout assists
const int xMargins = 5;
const int xOutdoorMargin = ((display.width() / 2) + xMargins);
const int yMargins = 2;
// yCO2 not used
const int yCO2 = 20;
const int ySparkline = 40;
const int yTemp = 100;
// BUG, 7/8 = 112, WiFi status is 15 (5*3) pixels high
const int yStatus = (display.height() * 7 / 8);
const int sparklineHeight = 40;
const int batteryBarWidth = 28;
const int batteryBarHeight = 10;
#endif

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

float pm25Total = 0;  // running total of humidity over report interval
float avgPM25;        // average PM2.5 over report interval

unsigned long prevReportMs = 0;  // Timestamp for measuring elapsed capture time
unsigned long prevSampleMs = 0;  // Timestamp for measuring elapsed sample time
unsigned int numSamples = 0;     // Number of overall sensor readings over reporting interval
unsigned int numReports = 0;     // Number of capture intervals observed
// used by thingspeak and dweet
float MinPm25 = 1999; /* Observed minimum PM2.5 */
float MaxPm25 = -99;  /* Observed maximum PM2.5 */

int rssi = 0;  // WiFi RSSI value

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
// MQTT uses WiFiClient class to create TCP connections
WiFiClient client;

// MQTT interface depends on the underlying network client object, which is defined and
// managed here (so needs to be defined here).
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
Adafruit_MQTT_Client pm25_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

// Adafruit PMSA003I
extern bool mqttDeviceWiFiUpdate(int rssi);
extern bool mqttSensorUpdate(float pm25, float aqi, float tempF, float vocIndex, float humidity);
#endif

void setup() {
  // handle Serial first so status messages and debugMessage() work
  Serial.begin(115200);
  // wait for serial port connection
  while (!Serial)
    ;  // FIX: Infinite loop if no serial connection

  // Display key configuration parameters
  debugMessage("PM2.5 monitor started", 1);
  debugMessage(String("Sample interval is ") + SAMPLE_INTERVAL + " seconds", 2);
  debugMessage(String("Report interval is ") + REPORT_INTERVAL + " minutes", 2);
  debugMessage(String("Internet service reconnect delay is ") + CONNECT_ATTEMPT_INTERVAL + " seconds", 2);

#ifdef SCREEN
  // there is no way to query screen for status
  display.begin(THINKINK_MONO);  // changed from THINKINK_GRAYSCALE4 to eliminate black screen border
  debugMessage("Display ready", 1);
#endif

  // Initialize environmental sensor
  if (!sensorInit()) {
    debugMessage("Environment sensor failed to initialize", 1);
    screenAlert("NO PM25 sensor");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  // Remember current clock time
  prevReportMs = prevSampleMs = millis();

  networkConnect();
}

void loop() 
{
  // update current timer value
  unsigned long currentMillis = millis();

  // is it time to read the sensor
  if ((currentMillis - prevSampleMs) >= (SAMPLE_INTERVAL * 1000))  // converting SAMPLE_INTERVAL into milliseconds
  {
    if (readSensor()) 
    {
      numSamples++;

      // convert temp from C to F
      sensorData.ambientTemperature = 32.0 + (1.8 * sensorData.ambientTemperature);

      // add to the running totals
      pm25Total += sensorData.massConcentrationPm2p5;
      tempFTotal += sensorData.ambientTemperature;
      humidityTotal += sensorData.ambientHumidity;
      vocTotal += sensorData.vocIndex;

      debugMessage("Current Readings: ",1);
      debugMessage(String("PM2.5: ") + sensorData.massConcentrationPm2p5 + " = AQI " + pm25toAQI(sensorData.massConcentrationPm2p5), 1);
      debugMessage(String(", Temp: ") + sensorData.ambientTemperature + " F",1);
      debugMessage(String(", Humidity:  ") + sensorData.ambientHumidity + "%",1);
      debugMessage(String(", VOC: ") + sensorData.vocIndex,1);
      debugMessage(String("Sample #") + numSamples + String(", running totals: "),1);
      debugMessage(String("PM25 total: ") + pm25Total,1);
      debugMessage(String(", TempF total: ") + tempFTotal,1);
      debugMessage(String(", Humidity total: ") + humidityTotal,1);
      debugMessage(String(", VOC total: ") + vocTotal,1);

      #ifdef SCREEN
        screenPM();
      #endif

      // Save sample time
      prevSampleMs = currentMillis;
    } 
    else 
    {
      debugMessage("Could not read SEN54 sensor data", 1);
    }
  }

  // do we have network endpoints to report to?
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
    // is it time to report to the network endpoints?
    if ((currentMillis - prevReportMs) >= (REPORT_INTERVAL * 60 * 1000))  // converting REPORT_INTERVAL into milliseconds
    {
      // do we have samples to report?
      if (numSamples != 0) 
      {
        avgPM25 = pm25Total / numSamples;
        if (avgPM25 > MaxPm25) MaxPm25 = avgPM25;
        if (avgPM25 < MinPm25) MinPm25 = avgPM25;

        avgTempF = tempFTotal / numSamples;
        avgVOC = vocTotal / numSamples;
        avgHumidity = humidityTotal / numSamples;

        debugMessage("----- Reporting -----",1);
        debugMessage(String("Reporting averages (") + REPORT_INTERVAL + String(" minute): "),1);
        debugMessage(String("PM2.5: ") + avgPM25 + String(" = AQI ") + pm25toAQI(avgPM25),1);
        debugMessage(String(", Temp: ") + avgTempF + String(" F"),1);
        debugMessage(String(", Humidity: ") + avgHumidity + String("%"),1);
        debugMessage(String(", VoC: ") + avgVOC,1);

        if (networkConnect())
        {
          /* Post both the current readings and historical max/min readings to the internet */
          #ifdef DWEET
            post_dweet(avgPM25, pm25toAQI(MinPm25), pm25toAQI(MaxPm25), pm25toAQI(avgPM25), avgTempF, avgVOC, avgHumidity);
          #endif

          // Also post the AQI sensor data to ThingSpeak
          #ifdef THINGSPEAK
            post_thingspeak(avgPM25, pm25toAQI(MinPm25), pm25toAQI(MaxPm25), pm25toAQI(avgPM25));
          #endif

          #ifdef INFLUX
            if (!post_influx(avgPM25, pm25toAQI(avgPM25), avgTempF, avgVOC, avgHumidity, rssi))
              debugMessage("Did not write to influxDB",1);
          #endif

          #ifdef MQTT
            if (!mqttDeviceWiFiUpdate(rssi))
              debugMessage("Did not write device data to MQTT broker",1);
            if (!mqttSensorUpdate(avgPM25, pm25toAQI(avgPM25), avgTempF, avgVOC, avgHumidity))
              debugMessage("Did not write environment data to MQTT broker",1);
          #endif
        }
        // Reset counters and accumulators
        prevReportMs = currentMillis;
        numSamples = 0;
        pm25Total = 0;
        tempFTotal = 0;
        vocTotal = 0;
        humidityTotal = 0;
      }
    }
  #endif
  else 
  {
    debugMessage("No samples to average and report on",2);
  }
}

void screenPM() {
#ifdef SCREEN
  debugMessage("Starting screenPM refresh", 1);

  display.clearBuffer();
  display.setTextColor(EPD_BLACK);

  // screen helper routines
  // -70 moves it to the left of the battery display
  screenHelperWiFiStatus((display.width() - xMargins - 70), (display.height() - yMargins), 3, 3, 5);

  display.setFont(&FreeSans18pt7b);
  display.setCursor(xMargins, yMargins);
  display.print(String("PM2.5: ") + sensorData.massConcentrationPm2p5);
  display.setCursor(xMargins, 30);
  display.print(String("Temp: ") + sensorData.ambientTemperature + " F");
  display.setCursor(xMargins, 60);
  display.print(String("Humidity:  ") + sensorData.ambientHumidity + "%");
  display.setCursor(xMargins, 90);
  display.print(String("VOC: ") + sensorData.vocIndex);

  //update display
  display.display();
  debugMessage("screenPM refresh complete", 1);
#endif
}

void screenAlert(String messageText)
// Display critical error message on screen
{
  display.clearBuffer();
  display.setTextColor(EPD_BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(40, (display.height() / 2 + 6));
  display.print(messageText);

  //update display
  display.display();
}

void screenHelperStatusMessage(int initialX, int initialY, String messageText)
// helper function for screenXXX() routines that draws a status message
// uses system default font, so text drawn x+,y+ from initialX,Y
{
// IMPROVEMENT : Screen dimension boundary checks for function parameters
#ifdef SCREEN
  display.setFont();  // resets to system default monospace font (6x8 pixels)
  display.setCursor(initialX, initialY);
  display.print(messageText);
#endif
}

void screenHelperWiFiStatus(int initialX, int initialY, int barWidth, int barHeightMultiplier, int barSpacingMultipler)
// helper function for screenXXX() routines that draws WiFi signal strength
{
#ifdef SCREEN
  if (rssi != 0) {
    // Convert RSSI values to a 5 bar visual indicator
    // >90 means no signal
    int barCount = constrain((6 - ((rssi / 10) - 3)), 0, 5);
    if (barCount > 0) {
      // <50 rssi value = 5 bars, each +10 rssi value range = one less bar
      // draw bars to represent WiFi strength
      for (int b = 1; b <= barCount; b++) {
        display.fillRect((initialX + (b * barSpacingMultipler)), (initialY - (b * barHeightMultiplier)), barWidth, b * barHeightMultiplier, EPD_BLACK);
      }
      debugMessage(String("WiFi signal strength on screen as ") + barCount + " bars", 2);
    } else {
      // you could do a visual representation of no WiFi strength here
      debugMessage("RSSI too low, no display", 1);
    }
  }
#endif
}

bool networkConnect() {
// Run only if using network data endpoints
#if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)

  // reconnect to WiFi only if needed
  if (WiFi.status() == WL_CONNECTED) 
  {
    debugMessage("Already connected to WiFi",2);
    return true;
  }
  // set hostname has to come before WiFi.begin
  WiFi.hostname(CLIENT_ID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
  // Attempts WiFi connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
  {
    if (WiFi.status() == WL_CONNECTED) {
      rssi = abs(WiFi.RSSI());
      debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString(), 1);
      debugMessage(String("WiFi RSSI is: ") + rssi + " dBm", 1);
      return true;
    }
    debugMessage(String("Connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " to " + WIFI_SSID + " failed", 1);
    // use of delay() OK as this is initialization code
    delay(CONNECT_ATTEMPT_INTERVAL * 1000);  // convered into milliseconds
  }
#endif
  return false;
}

bool sensorInit() {
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
  if (error) {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " error during SEN5x reset", 1);
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
    debugMessage(String(errorMessage) + " error setting temp offset", 1);
  } else {
    debugMessage(String("Temperature Offset set to ") + tempOffset + " degrees C", 2);
  }

  // Start Measurement
  error = sen5x.startMeasurement();
  if (error) {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " error during SEN5x startMeasurement", 1);
    return false;
  }
  debugMessage("SEN5x initialized", 1);
  return true;
}

bool readSensor() {
  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];

// If we're simulating the sensor generate some reasonable values for measurements
#ifdef SIMULATE_SENSOR
  sensorData.massConcentrationPm1p0 = random(0, 360) / 10.0;
  sensorData.massConcentrationPm2p5 = random(0, 360) / 10.0;
  sensorData.massConcentrationPm4p0 = random(0, 720) / 10.0;
  sensorData.massConcentrationPm10p0 = random(0, 1550) / 10.0;
  sensorData.ambientHumidity = 30.0 + (random(0, 250) / 10.0);
  sensorData.ambientTemperature = 15.0 + (random(0, 101) / 10.0);
  sensorData.vocIndex = random(0, 3500) / 10.0;
  sensorData.noxIndex = random(0, 2500) / 10.0;
  return true;
#endif

  error = sen5x.readMeasuredValues(
    sensorData.massConcentrationPm1p0, sensorData.massConcentrationPm2p5, sensorData.massConcentrationPm4p0,
    sensorData.massConcentrationPm10p0, sensorData.ambientHumidity, sensorData.ambientTemperature, sensorData.vocIndex,
    sensorData.noxIndex);
  if (error) {
    errorToString(error, errorMessage, 256);
    debugMessage(String(errorMessage) + " error during SEN5x read",1);
    return false;
  }
  return true;
}

float pm25toAQI(float pm25)
// Converts pm25 reading to AQI using the AQI Equation
// (https://forum.airnowtech.org/t/the-aqi-equation/169)
{
  if (pm25 <= 12.0) return (fmap(pm25, 0.0, 12.0, 0.0, 50.0));
  else if (pm25 <= 35.4) return (fmap(pm25, 12.1, 35.4, 51.0, 100.0));
  else if (pm25 <= 55.4) return (fmap(pm25, 35.5, 55.4, 101.0, 150.0));
  else if (pm25 <= 150.4) return (fmap(pm25, 55.5, 150.4, 151.0, 200.0));
  else if (pm25 <= 250.4) return (fmap(pm25, 150.5, 250.4, 201.0, 300.0));
  else if (pm25 <= 500.4) return (fmap(pm25, 250.5, 500.4, 301.0, 500.0));
  else return (505.0);  // AQI above 500 not recognized
}

float fmap(float x, float xmin, float xmax, float ymin, float ymax) {
  return (ymin + ((x - xmin) * (ymax - ymin) / (xmax - xmin)));
}

void networkDisconnect()
{
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
  {
    WiFi.disconnect();
    debugMessage("Disconnected from WiFi network",1);
  }
  #endif
}

void powerDisable(int deepSleepTime)
// Powers down hardware in preparation for board deep sleep
{
  #ifdef SCREEN
    debugMessage("Starting power down activities",1);
    // power down epd
    display.powerDown();
    digitalWrite(EPD_RESET, LOW);  // hardware power down mode
    debugMessage("powered down epd",1);
  #endif

  networkDisconnect();

  // power down sensor

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("disabled Adafruit Feather ESP32 V2 I2C power",1);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, LOW);

    // if you need to turn the neopixel off
    // pinMode(NEOPIXEL_POWER, OUTPUT);
    // digitalWrite(NEOPIXEL_POWER, LOW);
    debugMessage("disabled Adafruit Feather ESP32S2 I2C power",1);
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  debugMessage(String("Going into ESP32 deep sleep for ") + (deepSleepTime) + " seconds",1);
  esp_deep_sleep_start();
}

void debugMessage(String messageText, int messageLevel)
// wraps Serial.println as #define conditional
{
#ifdef DEBUG
  if (messageLevel <= DEBUG) {
    Serial.println(messageText);
    Serial.flush();  // Make sure the message gets output (before any sleeping...)
  }
#endif
}