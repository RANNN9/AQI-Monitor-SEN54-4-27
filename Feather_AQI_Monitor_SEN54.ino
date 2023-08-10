/*
  Project:      pm25
  Description:  Regularly sample and log PM 2.5 levels

  See README.md for target information
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
  // 1.8" TFT display with 128x160 pixels
  #include <Adafruit_GFX.h>    // Core graphics library
  #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
  Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

  #include <Fonts/FreeSans9pt7b.h>
  #include <Fonts/FreeSans12pt7b.h>
  #include <Fonts/FreeSans18pt7b.h>
#endif

// global variables

// PM sensor data
typedef struct envData
{
  float massConcentrationPm1p0;   // PM1.0 [µg/m³], NAN if unknown
  float massConcentrationPm2p5;   // PM2.5 [µg/m³], NAN if unknown
  float massConcentrationPm10p0;  // PM10.0 [µg/m³], NAN if unknown
  float massConcentrationPm4p0;   // PM4.0 [µg/m³], NAN if unknown
  float ambientHumidity;          // RH [%], NAN if unknown
  float ambientTemperatureF;      // [°C], NAN in unknown
  float vocIndex;                 // Sensiron VOC Index, NAN in unknown
  float noxIndex;                 // NAN for unsupported devices (SEN54), also NAN for first 10-11 seconds
} envData;
envData sensorData;

float humidityTotal = 0;  // running total of humidity over report interval
float temperatureFTotal = 0;     // running total of temperature over report interval
float vocTotal = 0;       // running total of VOC over report interval
float avgtemperatureF = 0;       // average temperature over report interval
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
  extern void post_dweet(float pm25, float minaqi, float maxaqi, float aqi, float temperatureF, float vocIndex, float humidity, int rssi);
#endif

#ifdef THINGSPEAK
  extern void post_thingspeak(float pm25, float minaqi, float maxaqi, float aqi);
#endif

#ifdef INFLUX
  extern bool post_influx(float pm25, float aqi, float temperatureF, float vocIndex, float humidity, int rssi);
#endif

#ifdef MQTT
  // MQTT uses WiFiClient class to create TCP connections
  WiFiClient client;

  // MQTT interface depends on the underlying network client object, which is defined and
  // managed here (so needs to be defined here).
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);

  extern bool mqttDeviceWiFiUpdate(int rssi);
  extern bool mqttSensorTemperatureFUpdate(float temperatureF);
  extern bool mqttSensorHumidityUpdate(float humidity);
  extern bool mqttSensorPM25Update(float pm25);
  extern bool mqttSensorAQIUpdate(float aqi);
  extern bool mqttSensorVOCIndexUpdate(float vocIndex);
  #ifdef HASSIO_MQTT
    extern void hassio_mqtt_publish(float pm25, float aqi, float temperatureF, float vocIndex, float humidity);
  #endif
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

  // Initialize environmental sensor
  if (!sensorInit()) {
    debugMessage("Environment sensor failed to initialize", 1);
    screenAlert("NO PM25 sensor");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    //powerDisable(HARDWARE_ERROR_INTERVAL);
  }

  #ifdef SCREEN
    display.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    display.setRotation(DISPLAY_ROTATION);
    display.fillScreen(ST77XX_BLACK);
  #endif

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
      sensorData.ambientTemperatureF = 32.0 + (1.8 * sensorData.ambientTemperatureF);

      // add to the running totals
      pm25Total += sensorData.massConcentrationPm2p5;
      temperatureFTotal += sensorData.ambientTemperatureF;
      humidityTotal += sensorData.ambientHumidity;
      vocTotal += sensorData.vocIndex;

      debugMessage("Current Readings: ",1);
      debugMessage(String("PM2.5: ") + sensorData.massConcentrationPm2p5 + " = AQI " + pm25toAQI(sensorData.massConcentrationPm2p5), 1);
      debugMessage(String("Temperature: ") + sensorData.ambientTemperatureF + " F",1);
      debugMessage(String("Humidity:  ") + sensorData.ambientHumidity + "%",1);
      debugMessage(String("VOC: ") + sensorData.vocIndex,1);
      debugMessage(String("Sample #") + numSamples + String(", running totals: "),1);
      debugMessage(String("PM25 total: ") + pm25Total,1);
      debugMessage(String("temperatureF total: ") + temperatureFTotal,1);
      debugMessage(String("Humidity total: ") + humidityTotal,1);
      debugMessage(String("VOC total: ") + vocTotal,1);

      screenPM();
      
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

        avgtemperatureF = temperatureFTotal / numSamples;
        avgVOC = vocTotal / numSamples;
        avgHumidity = humidityTotal / numSamples;

        debugMessage("----- Reporting -----",1);
        debugMessage(String("Reporting averages (") + REPORT_INTERVAL + String(" minute): "),1);
        debugMessage(String("PM2.5: ") + avgPM25 + String(" = AQI ") + pm25toAQI(avgPM25),1);
        debugMessage(String("Temp: ") + avgtemperatureF + String(" F"),1);
        debugMessage(String("Humidity: ") + avgHumidity + String("%"),1);
        debugMessage(String("VoC: ") + avgVOC,1);

        if (networkConnect())
        {
          /* Post both the current readings and historical max/min readings to the internet */
          #ifdef DWEET
            post_dweet(avgPM25, pm25toAQI(MinPm25), pm25toAQI(MaxPm25), pm25toAQI(avgPM25), avgtemperatureF, avgVOC, avgHumidity, rssi);
          #endif

          // Also post the AQI sensor data to ThingSpeak
          #ifdef THINGSPEAK
            post_thingspeak(avgPM25, pm25toAQI(MinPm25), pm25toAQI(MaxPm25), pm25toAQI(avgPM25));
          #endif

          #ifdef INFLUX
            if (!post_influx(avgPM25, pm25toAQI(avgPM25), avgtemperatureF, avgVOC, avgHumidity, rssi))
              debugMessage("Did not write to influxDB",1);
          #endif

          #ifdef MQTT
            if (!mqttDeviceWiFiUpdate(rssi))
                debugMessage("Did not write device data to MQTT broker",1);
            if ((!mqttSensorTemperatureFUpdate(avgtemperatureF)) || (!mqttSensorHumidityUpdate(avgHumidity)) || (!mqttSensorPM25Update(avgPM25)) || (!mqttSensorAQIUpdate(pm25toAQI(avgPM25))) || (!mqttSensorVOCIndexUpdate(avgVOC)))
                debugMessage("Did not write environment data to MQTT broker",1);
            #ifdef HASSIO_MQTT
              debugMessage("Establishing MQTT for Home Assistant",1);
              // Either configure sensors in Home Assistant's configuration.yaml file
              // directly or attempt to do it via MQTT auto-discovery
              // hassio_mqtt_setup();  // Config for MQTT auto-discovery
              hassio_mqtt_publish(avgPM25, pm25toAQI(avgPM25), avgtemperatureF, avgVOC, avgHumidity);
            #endif
          #endif
        }
        // Reset counters and accumulators
        prevReportMs = currentMillis;
        numSamples = 0;
        pm25Total = 0;
        temperatureFTotal = 0;
        vocTotal = 0;
        humidityTotal = 0;
      }
    }
  #endif
}

void screenPM() {
#ifdef SCREEN
  debugMessage("Starting screenPM refresh", 1);

  // clear screen
  display.fillScreen(ST77XX_BLACK);

  // screen helper routines
  screenHelperWiFiStatus((display.width() - xMargins - ((5*wifiBarWidth)+(4*wifiBarSpacing))), (yMargins + (5*wifiBarHeightIncrement)), wifiBarWidth, wifiBarHeightIncrement, wifiBarSpacing);

  // temperature and humidity
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(xMargins, yTemperature);
  display.print(sensorData.ambientTemperatureF,1);
  display.print("F ");
  if ((sensorData.ambientHumidity<40) || (sensorData.ambientHumidity>60))
    display.setTextColor(ST7735_RED);
  else
    display.setTextColor(ST7735_GREEN);
  display.print(sensorData.ambientHumidity,1);
  display.print("%");

  // pm25 level circle
  switch (int(sensorData.massConcentrationPm2p5/50))
  {
    case 0: // good
      display.fillCircle(46,75,31,ST77XX_BLUE);
      break;
    case 1: // moderate
      display.fillCircle(46,75,31,ST77XX_GREEN);
      break;
    case 2: // unhealthy for sensitive groups
      display.fillCircle(46,75,31,ST77XX_YELLOW);
      break;
    case 3: // unhealthy
      display.fillCircle(46,75,31,ST77XX_ORANGE);
      break;
    case 4: // very unhealthy
      display.fillCircle(46,75,31,ST77XX_RED);
      break;
    case 5: // very unhealthy
      display.fillCircle(46,75,31,ST77XX_RED);
      break;
    default: // >=6 is hazardous
      display.fillCircle(46,75,31,ST77XX_MAGENTA);
      break;
  }

  // pm25 legend
  display.fillRect(xMargins,yLegend,legendWidth,legendHeight,ST77XX_BLUE);
  display.fillRect(xMargins,yLegend-legendHeight,legendWidth,legendHeight,ST77XX_GREEN);
  display.fillRect(xMargins,(yLegend-(2*legendHeight)),legendWidth,legendHeight,ST77XX_YELLOW);
  display.fillRect(xMargins,(yLegend-(3*legendHeight)),legendWidth,legendHeight,ST77XX_ORANGE);
  display.fillRect(xMargins,(yLegend-(4*legendHeight)),legendWidth,legendHeight,ST77XX_RED);
  display.fillRect(xMargins,(yLegend-(5*legendHeight)),legendWidth,legendHeight,ST77XX_MAGENTA);


  // VoC level circle
  switch (int(sensorData.vocIndex/100))
  {
    case 0: // great
      display.fillCircle(114,75,31,ST77XX_BLUE);
      break;
    case 1: // good
      display.fillCircle(114,75,31,ST77XX_GREEN);
      break;
    case 2: // moderate
      display.fillCircle(114,75,31,ST77XX_YELLOW);
      break;
    case 3: // 
      display.fillCircle(114,75,31,ST77XX_ORANGE);
      break;
    case 4: // bad
      display.fillCircle(114,75,31,ST77XX_RED);
      break;
  }

  // VoC legend
  display.fillRect(display.width()-xMargins,yLegend,legendWidth,legendHeight,ST77XX_BLUE);
  display.fillRect(display.width()-xMargins,yLegend-legendHeight,legendWidth,legendHeight,ST77XX_GREEN);
  display.fillRect(display.width()-xMargins,(yLegend-(2*legendHeight)),legendWidth,legendHeight,ST77XX_YELLOW);
  display.fillRect(display.width()-xMargins,(yLegend-(3*legendHeight)),legendWidth,legendHeight,ST77XX_ORANGE);
  display.fillRect(display.width()-xMargins,(yLegend-(4*legendHeight)),legendWidth,legendHeight,ST77XX_RED);

  // circle labels
  display.setTextColor(ST77XX_WHITE);
  display.setFont();
  display.setCursor(33,110);
  display.print("PM2.5");
  display.setCursor(106,110);
  display.print("VoC"); 


  // pm25 level
  display.setFont(&FreeSans9pt7b);
  display.setCursor(40,80);
  display.print(int(sensorData.massConcentrationPm2p5));

  // VoC level
  display.setCursor(100,80);
  display.print(int(sensorData.vocIndex));
#endif
}

void screenAlert(String messageText)
// Display critical error message on screen
{
  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(40, (display.height() / 2 + 6));
  display.print(messageText);
}

void screenHelperWiFiStatus(int initialX, int initialY, int barWidth, int barHeightIncrement, int barSpacing)
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
        display.fillRect((initialX + (b * barSpacing)), (initialY - (b * barHeightIncrement)), barWidth, b * barHeightIncrement, ST77XX_WHITE);
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
  #ifdef SIMULATE_SENSOR
    // IMPROVEMENT : Could simulate IP address
    // testing range is 30 to 90 (no signal)
    rssi  = random(30, 90);
    return true;
  #endif

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

  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];

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
  // If we're simulating the sensor generate some reasonable values for measurements
  #ifdef SIMULATE_SENSOR
    sensorData.massConcentrationPm1p0 = random(0, 360) / 10.0;
    sensorData.massConcentrationPm2p5 = random(0, 360) / 10.0;
    sensorData.massConcentrationPm4p0 = random(0, 720) / 10.0;
    sensorData.massConcentrationPm10p0 = random(0, 1550) / 10.0;
    // testing range is 5 to 95
    sensorData.ambientHumidity = 5 + (random(0, 900) / 10.0);
    // keep this value in C, not F. Converted after readSensor()
    // testing range is 15 to 25
    sensorData.ambientTemperatureF = 15.0 + (random(0, 101) / 10.0);
    sensorData.vocIndex = random(0, 500) / 10.0;
    sensorData.noxIndex = random(0, 2500) / 10.0;
    return true;
  #endif

  // SparkFun SEN5X
  uint16_t error;
  char errorMessage[256];

  error = sen5x.readMeasuredValues(
    sensorData.massConcentrationPm1p0, sensorData.massConcentrationPm2p5, sensorData.massConcentrationPm4p0,
    sensorData.massConcentrationPm10p0, sensorData.ambientHumidity, sensorData.ambientTemperatureF, sensorData.vocIndex,
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